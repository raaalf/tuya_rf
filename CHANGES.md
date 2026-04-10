# Changes from original olivluca/tuya_rf

This is a stability-focused fork combining the best changes from multiple forks.
All changes were made after reviewing the complete source code of the original and 6 forks.

## Changes Applied

### 1. Transmit Queue System (from bachlv/tuya_rf)

**Problem**: Sending multiple RF commands in quick succession causes collisions.
The CMT2300A chip cannot TX and RX simultaneously, and switching modes takes time.
Without queuing, rapid commands can arrive before the chip has returned to a stable state.

**Solution**: Commands are buffered in a `std::deque` and sent one at a time with a
configurable delay between transmissions.

**Files modified**: `__init__.py`, `tuya_rf.h`, `tuya_rf.cpp`, `tuya_rf_libretiny.cpp`, `automation.h`

**New ESPHome config options**:
```yaml
tuya_rf:
  queue_max_size: 10    # 1-100, default 10
  queue_delay: 100ms    # max 10s, default 100ms — minimum gap between transmissions
```

**New ESPHome action**:
```yaml
# Use this instead of remote_transmitter.transmit_raw for RF:
- tuya_rf.queue_transmit:
    receiver_id: rf
    data: !lambda "return code;"
```

### 2. Stabilization Delays in CMT2300A Mode Transitions (inspired by hoducnguyenhd/tuya_rf)

**Problem**: The original code switches CMT2300A states (Sleep -> Standby -> TX/RX)
without delays. The chip may not have fully settled before the next transition begins,
causing intermittent TX failures.

**Solution**: Added 2ms `delay()` calls at critical state transitions in `radio.c`:
- After `GoStby()` before `GoTx()` — let chip settle in Standby
- After `GoTx()` succeeds — let chip fully enter TX mode
- After clearing RX FIFO before `GoRx()` — let FIFO clear complete
- After `GoRx()` — let chip fully enter RX mode

**Important**: Unlike hoducnguyenhd's fork, we keep the original GPIO configuration,
register bank loading, and RF parameters from the captured Tuya firmware. Only the
delays are added — no changes to radio configuration.

**Files modified**: `radio.c`

## Forks Analyzed (source code review)

| Fork | Changes | Included Here |
|------|---------|--------------|
| olivluca (original) | Base code | Yes (base) |
| amirlanesman | RemoteReceiverBase fix (already merged to original) | Already in base |
| bachlv | Transmit queue system | Yes |
| kar200 | No changes (exact clone) | N/A |
| hoducnguyenhd | Rewritten radio.c + different RF params | Delay idea only |
| Rulezt | GPIO3 mask cleanup in StartTx() | No (cosmetic) |
| aalaei | No changes (exact clone) | N/A |

## Recommended ESPHome Configuration

See `tuya_avatto_s16_pro.yaml` for a complete example configuration for the Avatto S16 Pro.
