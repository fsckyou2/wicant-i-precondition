#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "hsm.h"

#define TAG "hsm"

// fill path[0..n-1] with the ancestor chain of s, outermost first; returns n
static int ancestor_path(const sm_state_t *s, const sm_state_t *path[SM_MAX_DEPTH]) {
    int n = 0;
    for (const sm_state_t *p = s; p != NULL; p = p->parent) {
        n++;
    }
    if (n > SM_MAX_DEPTH) {
        ESP_LOGE(TAG, "state %s exceeds SM_MAX_DEPTH", s->name);
        return 0;
    }
    int i = n;
    for (const sm_state_t *p = s; p != NULL; p = p->parent) {
        path[--i] = p;
    }
    return n;
}

static void do_transition(sm_t *sm, const sm_state_t *target) {
    const sm_state_t *src_path[SM_MAX_DEPTH];
    const sm_state_t *dst_path[SM_MAX_DEPTH];
    int ns = ancestor_path(sm->current, src_path);
    int nd = ancestor_path(target, dst_path);
    const char *from_name = sm->current->name;

    // depth of the first level where the two paths diverge; everything above
    // it stays active through the transition
    int keep = 0;
    while (keep < ns && keep < nd && src_path[keep] == dst_path[keep]) {
        keep++;
    }
    // external-transition rule: when one state contains the other (including
    // a self-transition), exit and re-enter the inner one instead of no-op
    if ((keep == ns || keep == nd) && keep > 0) {
        keep--;
    }

    for (int i = ns - 1; i >= keep; i--) {
        if (src_path[i]->exit != NULL) {
            src_path[i]->exit(sm);
        }
    }

    for (int i = keep; i < nd; i++) {
        sm->entered_ts[i] = sm->now;
        if (dst_path[i]->enter != NULL) {
            dst_path[i]->enter(sm);
        }
    }

    // descend through default children until we reach a leaf
    const sm_state_t *leaf = target;
    int depth = nd;
    while (leaf->initial != NULL && depth < SM_MAX_DEPTH) {
        leaf = leaf->initial;
        sm->entered_ts[depth] = sm->now;
        if (leaf->enter != NULL) {
            leaf->enter(sm);
        }
        depth++;
    }

    sm->current = leaf;
    sm->leaf_ticks = 0;
    ESP_LOGI(sm->tag, "%s -> %s", from_name, leaf->name);
}

// apply any transition requested by a handler; returns true if one happened
// (i.e. the caller should stop dispatching the current tick/event/frame)
static bool apply_if_pending(sm_t *sm) {
    if (sm->pending == NULL) {
        return false;
    }
    // enter/exit handlers may request follow-up transitions
    int guard = 8;
    while (sm->pending != NULL && guard-- > 0) {
        const sm_state_t *target = sm->pending;
        sm->pending = NULL;
        do_transition(sm, target);
    }
    if (sm->pending != NULL) {
        ESP_LOGE(sm->tag, "transition loop; dropping transition to %s", sm->pending->name);
        sm->pending = NULL;
    }
    return true;
}

void sm_init(sm_t *sm, const char *tag, const sm_state_t *initial, const sm_hooks_t *global) {
    memset(sm, 0, sizeof(*sm));
    sm->tag = tag;
    sm->global = global;
    sm->now = esp_timer_get_time();

    // enter the initial state from the root down, then descend to a leaf
    const sm_state_t *path[SM_MAX_DEPTH];
    int n = ancestor_path(initial, path);
    for (int i = 0; i < n; i++) {
        sm->entered_ts[i] = sm->now;
        if (path[i]->enter != NULL) {
            path[i]->enter(sm);
        }
    }
    const sm_state_t *leaf = initial;
    while (leaf->initial != NULL && n < SM_MAX_DEPTH) {
        leaf = leaf->initial;
        sm->entered_ts[n] = sm->now;
        if (leaf->enter != NULL) {
            leaf->enter(sm);
        }
        n++;
    }
    sm->current = leaf;
    apply_if_pending(sm);
    ESP_LOGI(sm->tag, "init -> %s", sm->current->name);
}

void sm_tick(sm_t *sm) {
    if (sm->current == NULL) {
        return;
    }
    sm->now = esp_timer_get_time();
    if (sm->global != NULL && sm->global->tick != NULL) {
        sm->global->tick(sm);
        if (apply_if_pending(sm)) {
            return;
        }
    }
    const sm_state_t *path[SM_MAX_DEPTH];
    int n = ancestor_path(sm->current, path);
    for (int i = 0; i < n; i++) {
        if (path[i]->tick != NULL) {
            path[i]->tick(sm);
            if (apply_if_pending(sm)) {
                return;
            }
        }
    }
    sm->leaf_ticks++;
}

void sm_rx(sm_t *sm, const twai_message_t *msg, can_bus_t bus) {
    if (sm->current == NULL) {
        return;
    }
    sm->now = esp_timer_get_time();
    if (sm->global != NULL && sm->global->rx != NULL) {
        sm->global->rx(sm, msg, bus);
        if (apply_if_pending(sm)) {
            return;
        }
    }
    for (const sm_state_t *s = sm->current; s != NULL; s = s->parent) {
        if (s->rx != NULL) {
            s->rx(sm, msg, bus);
            if (apply_if_pending(sm)) {
                return;
            }
        }
    }
}

fwd_result_t sm_fwd(sm_t *sm, twai_message_t *msg, can_bus_t bus) {
    if (sm->current == NULL) {
        return FWD_PASSTHROUGH;
    }
    sm->now = esp_timer_get_time();
    fwd_result_t result = FWD_PASSTHROUGH;
    if (sm->global != NULL && sm->global->fwd != NULL) {
        result = sm->global->fwd(sm, msg, bus);
    }
    for (const sm_state_t *s = sm->current; result == FWD_PASSTHROUGH && s != NULL; s = s->parent) {
        if (s->fwd != NULL) {
            result = s->fwd(sm, msg, bus);
        }
    }
    if (sm->pending != NULL) {
        ESP_LOGW(sm->tag, "transition requested from a fwd handler");
        apply_if_pending(sm);
    }
    return result;
}

void sm_send_event(sm_t *sm, sm_event_t ev) {
    if (sm->current == NULL) {
        return;
    }
    sm->now = esp_timer_get_time();
    for (const sm_state_t *s = sm->current; s != NULL; s = s->parent) {
        if (s->event == NULL) {
            continue;
        }
        bool handled = s->event(sm, ev);
        if (apply_if_pending(sm)) {
            return;
        }
        if (handled) {
            return;
        }
    }
}

void sm_transition(sm_t *sm, const sm_state_t *target) {
    if (sm->pending != NULL && sm->pending != target) {
        ESP_LOGW(sm->tag, "overriding pending transition %s with %s",
                 sm->pending->name, target->name);
    }
    sm->pending = target;
}

bool sm_in(const sm_t *sm, const sm_state_t *state) {
    for (const sm_state_t *s = sm->current; s != NULL; s = s->parent) {
        if (s == state) {
            return true;
        }
    }
    return false;
}

int64_t sm_time_in_us(const sm_t *sm, const sm_state_t *state) {
    const sm_state_t *path[SM_MAX_DEPTH];
    int n = ancestor_path(sm->current, path);
    for (int i = 0; i < n; i++) {
        if (path[i] == state) {
            return sm->now - sm->entered_ts[i];
        }
    }
    return 0;
}

uint32_t sm_ticks_in_state(const sm_t *sm) {
    return sm->leaf_ticks;
}

int64_t sm_now(const sm_t *sm) {
    return sm->now;
}
