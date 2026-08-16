#ifndef __HSM_H__
#define __HSM_H__

// Small hierarchical state machine engine for CAN-hook-driven features.
//
// States are static const structs linked into a tree via .parent, with
// composite states naming their default child via .initial. The machine's
// current state is always a leaf. All callbacks are optional (NULL = skip).
//
// Dispatch rules:
//   sm_tick        global tick first, then ancestors outermost -> leaf.
//   sm_rx          global rx first, then states leaf -> outermost.
//   sm_send_event  states leaf -> outermost; stops at the first handler that
//                  returns true (handled) or requests a transition.
//   sm_fwd         global fwd first, then states leaf -> outermost; the first
//                  non-FWD_PASSTHROUGH result wins.
//
// Transitions are deferred: sm_transition() only records the target, and the
// engine applies it after the current handler returns, skipping the remaining
// handlers for that dispatch. Exception: after a transition inside
// sm_send_event, an enclosing sm_tick/sm_rx dispatch continues against the
// new state (so e.g. a burst state entered by an event still sends its first
// frame on the same tick). Exit handlers run leaf -> least common ancestor
// (exclusive), then enter handlers run down to the target and on through
// .initial children to a leaf. A transition to the current state or one of
// its ancestors exits and re-enters it (resetting its timers). enter/exit
// handlers may themselves call sm_transition; the engine applies it after the
// in-progress transition completes.
//
// Per-state context: a state may declare a context struct via .ctx/.ctx_size.
// The engine zeroes it whenever the state is entered from outside itself
// (before its enter handler runs), so context owned by a state and its
// descendants provably starts fresh each episode and is never stale across
// visits. Transitions within the owning state's subtree leave it untouched.
//
// Entry arguments: sm_transition_arg() attaches one intptr_t to a transition;
// the target's enter handlers read it via sm_entry_arg() — after the engine's
// context zeroing, so presets survive. Plain sm_transition() passes 0. The
// value is only meaningful inside enter handlers for that transition.
//
// Concurrency: the engine is not thread-safe. All calls into one sm_t must
// come from a single task (for preconditioning, the CAN RX task).

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "driver/twai.h"
#include "can.h"

// decision for a single frame about to be forwarded/bridged to a bus
typedef enum {
    FWD_BLOCK,
    FWD_MODIFIED,
    FWD_PASSTHROUGH,
} fwd_result_t;

// maximum nesting depth of a state (root-level state = depth 1)
#define SM_MAX_DEPTH 4

// event identifiers are defined by each state machine's owner
typedef int sm_event_t;

typedef struct sm sm_t;
typedef struct sm_state sm_state_t;

struct sm_state {
    const char *name;          // used in transition logs
    const sm_state_t *parent;  // NULL for root-level states
    const sm_state_t *initial; // composite states: default child; NULL for leaves
    void *ctx;                 // optional context zeroed by the engine on entry
    size_t ctx_size;
    void (*enter)(sm_t *sm);
    void (*exit)(sm_t *sm);
    void (*tick)(sm_t *sm);
    // return true to stop the event from bubbling to the parent state
    bool (*event)(sm_t *sm, sm_event_t ev);
    void (*rx)(sm_t *sm, const twai_message_t *msg, can_bus_t bus);
    // must not call sm_transition; fwd handlers are pure frame filters
    fwd_result_t (*fwd)(sm_t *sm, twai_message_t *msg, can_bus_t bus);
};

// callbacks that run unconditionally, before any per-state dispatch
typedef struct {
    void (*tick)(sm_t *sm);
    void (*rx)(sm_t *sm, const twai_message_t *msg, can_bus_t bus);
    fwd_result_t (*fwd)(sm_t *sm, twai_message_t *msg, can_bus_t bus);
} sm_hooks_t;

struct sm {
    const char *tag;               // log tag / machine name
    const sm_state_t *current;     // always a leaf; NULL until sm_init
    const sm_hooks_t *global;
    const sm_state_t *pending;     // transition requested during dispatch
    intptr_t pending_arg;          // entry argument for the pending transition
    intptr_t entry_arg;            // entry argument of the transition being applied
    int64_t now;                   // timestamp of the dispatch in progress
    int64_t entered_ts[SM_MAX_DEPTH]; // entry time per depth (0 = outermost)
    uint32_t leaf_ticks;           // completed tick dispatches since leaf entry
};

void sm_init(sm_t *sm, const char *tag, const sm_state_t *initial, const sm_hooks_t *global);
void sm_tick(sm_t *sm);
void sm_rx(sm_t *sm, const twai_message_t *msg, can_bus_t bus);
fwd_result_t sm_fwd(sm_t *sm, twai_message_t *msg, can_bus_t bus);
void sm_send_event(sm_t *sm, sm_event_t ev);
// request a transition; applied when the current handler returns (see above)
void sm_transition(sm_t *sm, const sm_state_t *target);
// like sm_transition, but attaches an entry argument for the target's enter
// handlers to read via sm_entry_arg()
void sm_transition_arg(sm_t *sm, const sm_state_t *target, intptr_t arg);
// the entry argument of the transition in progress; only meaningful inside
// enter handlers (0 for sm_transition and machine init)
intptr_t sm_entry_arg(const sm_t *sm);

// is `state` the current leaf or one of its ancestors?
bool sm_in(const sm_t *sm, const sm_state_t *state);
// time spent in `state` (current leaf or ancestor; 0 if not currently active)
int64_t sm_time_in_us(const sm_t *sm, const sm_state_t *state);
// number of sm_tick dispatches completed since the current leaf was entered:
// a tick handler sees 0 on its first call after entry
uint32_t sm_ticks_in_state(const sm_t *sm);
// timestamp taken at the start of the dispatch in progress
int64_t sm_now(const sm_t *sm);

#endif
