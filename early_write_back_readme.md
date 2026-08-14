# LLC Early Writeback

## 1. Purpose

This feature lets a caller request that a dirty line already present in the
last-level cache (LLC) be written to DRAM before normal LLC eviction selects
it.

The line stays in the LLC. The operation only changes whether the line is
considered dirty and whether an early writeback is waiting for space in the
DRAM write queue.

## 11. Debug probe added in `src/main.cc`

The simulator now includes a small validation harness that periodically selects
an LLC line at random and calls `early_clean_llc()` on it. This does not
change the early-writeback policy itself. It only adds instrumentation so the
implementation can be exercised and inspected while the simulator runs.

### What was added

#### Periodic probe trigger

The main simulation loop now calls a probe helper after `uncore.LLC.operate()`.
The probe runs every 1000 global cycles.

Why this was done:

- The probe needs to run often enough to exercise the code path repeatedly.
- It also needs to stay throttled so the extra logging does not dominate the
    simulation output or add unnecessary overhead.
- Running it from the main loop keeps the test close to the existing LLC
    timing model, so the probe observes the same cache state that normal traffic
    sees.

#### Live LLC line selection

The probe scans the current LLC contents in `uncore.LLC.block[set][way]` and
builds a candidate list from valid resident lines. It prefers dirty lines when
they exist, because dirty lines are the only ones that can meaningfully test
the early-clean behavior.

Why this was done:

- The existing `llc_lines` set is insert-only in the current codebase, so it is
    not a reliable source of live LLC residency.
- Scanning the real cache array avoids stale addresses, replaced lines, and
    invalid entries.
- Preferring dirty lines makes the probe more useful because it naturally
    drives the success path or the WQ-full retry path.

#### Dedicated debug log file

The probe writes records to `results_50M/early_clean_debug.log` instead of
stdout.

Why this was done:

- A separate file keeps the validation output easy to inspect after a run.
- It avoids mixing the new probe messages with the simulator’s normal console
    output.
- The file can be diffed or grepped across runs when comparing behavior.

#### Per-probe state logging

Each log entry records:

- the current cycle;
- the selected LLC address and line address;
- the set and way;
- the valid, dirty, and `early_write_back` state before the call;
- the lower-level WQ occupancy before and after the call;
- the return value from `early_clean_llc()`;
- the dirty and `early_write_back` state after the call.

Why this was done:

- The before/after state makes it easy to confirm whether the function actually
    cleaned the line.
- The WQ occupancy tells you whether a rejection happened because the write
    queue was full.
- Recording the selected set and way lets you correlate the probe with the
    cache contents if you need to inspect a specific resident line.
- Logging the return value gives a direct check on whether the function
    accepted the writeback or deferred it.

#### Early-clean call path stays unchanged

The implementation in `src/cache.cc` was not rewritten for this validation
work. The probe calls the existing `early_clean_llc()` entry point directly.

Why this was done:

- The goal is to validate the current implementation, not replace it.
- Reusing the existing function exercises the same logic the simulator would
    use in a real policy decision.
- Keeping the policy code unchanged reduces the risk of introducing a second
    behavior while debugging the first one.

### Current probe flow

```text
Global cycle reaches probe interval
                |
                v
Scan live LLC lines
                |
                v
Choose a random valid line, preferring dirty lines when available
                |
                v
Log pre-call state and WQ occupancy
                |
                v
Call uncore.LLC.early_clean_llc(full_addr)
                |
                v
Log return value and post-call state
```

### How to use the probe output

If `early_clean_llc()` works as expected, the log should show one of these
patterns:

- dirty line with WQ space: the line becomes clean and `accepted=1`;
- dirty line with WQ full: `early_write_back` stays set and `accepted=0`;
- already-clean line: `accepted=1` with no dirty-bit change;
- absent line: the probe skips it by construction because it only samples
    valid residents.

This means the probe is primarily a functional sanity check, not a policy
benchmark. Its purpose is to prove that the existing early-clean logic is being
hit, that it makes the expected state transitions, and that the retry behavior
is observable when the write queue is full.

## 12. Validation

The implementation should be checked with:

This version intentionally uses a simple address-based design:

- there is no request ID;
- there is no DRAM completion callback;
- there is no comparison with data read back from DRAM;
- a writeback is considered accepted when it enters the DRAM write queue.

The per-line state field is named `early_write_back`.

## 2. Public API

The request function is:

```cpp
bool CACHE::early_clean_llc(uint64_t full_addr);
```

The function name remains `early_clean_llc()` for compatibility with the
original early-clean API. The state bit and retry mechanism use the clearer
`early_write_back` name.

The function must be called on the LLC and must receive a byte address:

```cpp
bool accepted = uncore.LLC.early_clean_llc(full_addr);
```

Return values:

| Return value | Meaning |
| --- | --- |
| `true` | The line was absent, already clean, or its writeback was inserted into the lower-level WQ. |
| `false` | The line is dirty but the DRAM WQ is full. The request was saved in the line state and will be retried automatically. |

The function does not need to be called repeatedly after it returns `false`.
The LLC retry path checks pending lines during every LLC operation cycle.

## 3. State added to each cache block

`BLOCK` now contains:

```cpp
uint8_t early_write_back;
```

Its meaning is:

```text
early_write_back = 0
    No early writeback is waiting for DRAM WQ space.

early_write_back = 1
    The line is dirty and an early writeback request is waiting to be
    inserted into the DRAM write queue.
```

The bit is initialized to zero when a block is constructed.

No ID is stored in the block or in a packet. The current line address and the
current cache state are sufficient for this simplified behavior.

## 4. Request pipeline

The complete pipeline is:

```text
Caller passes a byte address
            |
            v
Find the line in the LLC
            |
            +-- line absent ----------------------> return true
            |
            +-- line clean ------------------------> return true
            |
            +-- line dirty, DRAM WQ full ----------> set early_write_back = 1
            |                                         keep dirty = 1
            |                                         return false
            |
            `-- line dirty, DRAM WQ has space
                                                      create WRITEBACK packet
                                                      add packet to DRAM WQ
                                                      clear early_write_back
                                                      clear dirty
                                                      return true
```

The line is never invalidated by this operation. Its tag, data, valid bit, and
replacement state remain in the LLC.

## 5. Detailed implementation changes

### `inc/block.h`

Added the `early_write_back` bit to `BLOCK` and initialized it to zero.

Why: the bit records a request that could not yet be sent because the lower
level write queue was full.

The old ID-related fields are not present in this design. There is no
`early_clean_id` in `BLOCK` or `PACKET`.

### `inc/cache.h`

Added:

```cpp
bool early_clean_llc(uint64_t full_addr);
void retry_early_writebacks();
```

`early_clean_llc()` handles a new request or retries a line that is already
marked with `early_write_back`.

`retry_early_writebacks()` scans LLC sets and ways for pending lines and calls
the request function again using the line's current full address.

### `src/cache.cc`: early-writeback request

`early_clean_llc()` performs these steps:

1. Assert that the cache is the LLC.
2. Convert the supplied byte address to a cache-line address.
3. Find the matching set and way.
4. Return immediately if the line is absent.
5. Clear a stale pending bit and return if the line is already clean.
6. Check the lower-level DRAM WQ occupancy.
7. If the WQ is full, set `early_write_back = 1`, leave `dirty = 1`, record the WQ-full event, and return `false`.
8. If the WQ has space, build a normal `WRITEBACK` packet using the line's current data.
9. Add the packet to the lower-level WQ.
10. Clear both `early_write_back` and `dirty`.

The writeback packet is ordinary cache traffic:

```cpp
writeback_packet.type = WRITEBACK;
```

No special packet fields are required.

### `src/cache.cc`: retry mechanism

`retry_early_writebacks()` runs from `CACHE::operate()` for the LLC:

```cpp
handle_fill();
handle_writeback();
retry_early_writebacks();
handle_read();
handle_prefetch();
```

It runs after `handle_writeback()` so that a newer writeback arriving in the
same cycle can cancel a pending early writeback before the retry scan runs.

For every valid LLC block, it checks:

```cpp
early_write_back == 1
dirty == 1
```

If both are true, it retries using the line's current data. A full DRAM WQ
leaves the state unchanged. Available WQ space causes the writeback to be
queued and both bits to be cleared.

### `src/cache.cc`: newer data cancels a pending request

When a writeback hits an existing LLC line, the code clears:

```cpp
block[set][way].early_write_back = 0;
```

before setting:

```cpp
block[set][way].dirty = 1;
```

This means newer data cancels an early request that has not yet entered the
DRAM WQ. The old data is never sent by the retry mechanism.

The writeback-miss/fill path also clears the bit. `fill_cache()` resets it when
a way is replaced, and the subsequent writeback fill marks the new line dirty
when appropriate.

### `src/cache.cc`: replacement and fill reset

`fill_cache()` now resets:

```cpp
block[set][way].dirty = 0;
block[set][way].early_write_back = 0;
```

This handles normal replacement and prevents pending state from belonging to
a new line occupying the same way.

## 6. All important cases

### Case 1: Missing line

The address is not resident in the LLC. No packet is generated and no state
is changed.

### Case 2: Already-clean line

No packet is generated. The function returns `true`.

### Case 3: Dirty line and WQ space available

The current data is copied into a normal writeback packet and inserted into
the DRAM WQ. The line remains resident and becomes clean immediately after
queue insertion.

### Case 4: Dirty line and WQ full

The line is marked:

```text
early_write_back = 1
dirty = 1
```

No stale packet is created. The retry scan tries again on later LLC cycles.

### Case 5: WQ becomes available later

The retry scan finds the pending line, creates a packet from the line's
current data, inserts it, and clears both state bits.

### Case 6: New data arrives before the request is queued

The LLC writeback-hit or writeback-fill path clears `early_write_back` and
sets `dirty = 1`. The pending request is discarded. The newer data can later
be early-cleaned by a new call.

### Case 7: New data arrives after the early writeback is queued

The simplified design has already cleared the dirty bit. A later writeback to
the LLC sets `dirty = 1` again and leaves `early_write_back = 0`.

No completion acknowledgement is used. The earlier writeback and newer
writeback are treated as ordinary queued memory operations.

### Case 8: Pending line is selected by normal LLC eviction

The normal eviction path takes priority because `handle_fill()` runs before
the retry scan.

If the lower-level WQ is full, the existing eviction path stalls with
`do_fill = 0` and retries later.

If the lower-level WQ has space, the normal victim writeback is queued and the
way is replaced. `fill_cache()` clears both `dirty` and `early_write_back`.

### Case 9: Multiple pending lines

The retry function scans all LLC sets and ways. It attempts each pending line
in order. If the DRAM WQ becomes full during the scan, remaining lines stay
pending for a later cycle.

## 7. Relationship with existing DRAM stalls

The existing simulator behavior is reused:

- lower-level WQ occupancy is checked before a dirty victim is replaced;
- a full WQ prevents the replacement and leaves the request pending;
- WQ-full and stall counters are updated;
- DRAM removes entries only after its normal write processing.

Early writeback uses the same occupancy check and WQ insertion mechanism. The
only difference is that a resident LLC line records its pending request in
`early_write_back` while it waits for capacity.

## 8. How and where to call it

Call the function from the component that decides that a particular LLC line
should be cleaned early. The argument must be a byte address:

```cpp
uint64_t address = ...;
bool accepted = uncore.LLC.early_clean_llc(address);
```

The function is appropriate for a policy decision point such as:

- a persistence or flush-like simulator event;
- a memory-pressure policy;
- a research policy that selects dirty LLC lines;
- an explicit test or instrumentation hook.

Do not call it from the ordinary L1 or L2 cache objects. The function asserts
that it is operating on the LLC.

The caller only needs to react specially to `false` if it wants to know that
the line is still waiting for WQ space. The LLC automatically retries pending
lines from `operate()`.

## 9. Timing and completion model

This implementation considers the operation complete when the writeback is
accepted into the DRAM WQ:

```text
LLC creates packet
        |
        v
DRAM WQ accepts packet
        |
        v
LLC clears dirty bit
```

The later DRAM scheduling and write-processing latency still affect when the
data reaches the simulated memory system, but they do not cause another LLC
callback or dirty-bit transition.

This is intentionally simpler than an acknowledgement-based persistence
model. It also removes the need for packet IDs, completion matching, and
DRAM-to-LLC notification.

## 10. Validation

The implementation should be checked with:

```bash
git diff --check
make -j2
```

The most important behavioral checks are:

1. A clean line creates no writeback.
2. A dirty line with WQ space creates one writeback and becomes clean.
3. A dirty line with a full WQ becomes pending and remains dirty.
4. A pending line retries after WQ space becomes available.
5. A newer LLC write cancels the pending bit and keeps the line dirty.
6. Normal eviction still stalls when its lower-level WQ is full.

## 12. Statistics and observability corrections

The random probe and the LLC statistics measure different kinds of events.
An early-clean request is a background writeback to DRAM; it is not a normal
LLC load, RFO, prefetch, or cache miss. Therefore, early cleaning must not be
added to the ordinary LLC hit/miss counters. Separate counters are used for
early-clean activity.

### 12.1 Correct LLC eviction classification

Earlier accounting placed the clean-victim test inside `if (victim.dirty)`.
That made the clean branch unreachable. Consequently, `TOTAL_EVICTIONS` was
always equal to `DIRTY_EVICTIONS`, and `CLEAN_EVICTIONS` stayed zero even when
clean lines were replaced.

The correction is implemented through `record_llc_eviction()` in
`src/cache.cc`. It runs only after a replacement can actually proceed and
classifies every valid LLC victim:

```text
valid victim -> TOTAL_EVICTIONS++
dirty victim -> DIRTY_EVICTIONS++ and TOTAL_DIRTY_WRITEBACKS++
clean victim -> CLEAN_EVICTIONS++
```

The helper is called from both replacement paths:

1. A normal LLC fill caused by a demand/refill miss.
2. An LLC writeback miss that selects a victim.

If the lower-level DRAM WQ is full, the replacement is stalled and the victim
is not counted yet. This prevents a blocked replacement from being reported as
an eviction that never happened.

### 12.2 Meaning of dirty-writeback counters

`TOTAL_DIRTY_WRITEBACKS` counts dirty victims written out because of normal
LLC replacement. It does not include early-clean requests. Early-clean traffic
has its own counters, so the two mechanisms can be compared without mixing
them.

DRAM completion timing is now recorded separately for every LLC writeback
packet that carries a writeback start cycle. The packet receives its start
cycle when it is generated, and the DRAM controller reports completion when
the WQ request finishes. `AVERAGE_DIRTY_WRITEBACK_LATENCY` is therefore the
time from writeback generation until DRAM completion; it is not merely the
time until the packet is accepted by the WQ.

The existing dirty-line lifetime accounting is also completed when an early
clean succeeds. The line was dirty until the early-clean request was accepted,
so its lifetime ends at that point rather than waiting for a later eviction.

### 12.3 Early-clean counters

The LLC now reports these values:

- `EARLY_CLEAN_ATTEMPTS`: calls to `early_clean_llc()`, including retries.
- `EARLY_CLEAN_DIRTY_SELECTED`: random probes that selected a dirty line.
- `EARLY_CLEAN_CLEAN_SELECTED`: random probes that selected a clean line.
- `EARLY_CLEAN_NO_LINE`: calls whose address no longer mapped to a valid line.
- `EARLY_CLEAN_CLEAN_NOOPS`: clean-line requests that correctly did nothing.
- `EARLY_CLEAN_QUEUED_OR_MERGED`: dirty requests accepted by the DRAM WQ.
- `EARLY_CLEAN_DEFERRED_WQ_FULL`: dirty requests delayed because the WQ was full.

`QUEUED_OR_MERGED` uses this name because the DRAM controller may merge a
duplicate writeback instead of increasing occupancy. Both cases mean that the
request was accepted and the LLC line can be marked clean under the current
queue-acceptance model.

### 12.4 Probe log result field

The probe log keeps the old `accepted` field for compatibility and adds a
clear `result` field:

```text
result=queued_or_merged
result=deferred_wq_full
result=clean_noop
```

`accepted=1` alone does not mean that a writeback was sent. For a clean line,
`accepted=1` means that the request was a successful no-op. The `result` field
removes that ambiguity.

The log records WQ occupancy before and after the request, the dirty and
early-writeback bits before and after the request, and the selected set/way.
This makes it possible to verify every state transition directly.

### 12.5 Log location

The probe log is written below the simulation-size directory, for example:

```text
results_1M/early_clean_debug.log
results_50M/early_clean_debug.log
```

The path is derived from `simulation_instructions`, instead of always using
`results_50M`. This prevents runs with different simulation lengths from
sharing or overwriting the wrong log.

### 12.6 Interpreting a zero pending-bit count

`early_write_back` becomes `1` only when a dirty line is selected while the
DRAM WQ is full. A run with `WQ FULL: 0` should therefore show:

```text
EARLY CLEAN DEFERRED WQ FULL: 0
```

and no probe records with `early_after=1`. This is expected, not evidence that
the early-clean call is inactive.

### 12.7 Validation checklist

For a meaningful validation run, check all of the following:

1. `CLEAN_EVICTIONS + DIRTY_EVICTIONS == TOTAL_EVICTIONS`.
2. `EARLY_CLEAN_DIRTY_SELECTED` matches the number of dirty probe records.
3. `EARLY_CLEAN_CLEAN_NOOPS` matches clean probe records.
4. `EARLY_CLEAN_DEFERRED_WQ_FULL` is nonzero before expecting
   `early_after=1`.
5. `result=queued_or_merged` records change `dirty_before=1` to
   `dirty_after=0`.
6. `result=clean_noop` records keep both dirty values at zero and do not
   increase WQ occupancy.
7. `AVERAGE_DIRTY_WRITEBACK_LATENCY` is populated only when a writeback reaches
   DRAM completion; a dash or zero means that no measured writeback completed
   in the selected statistics interval.
