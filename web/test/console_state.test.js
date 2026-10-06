import test from 'node:test'
import assert from 'node:assert/strict'
import { competitionEligibility, formatPoseSpeed, mergeDiagnostics,
  navigationMessage, targetRejectionMessage } from '../src/console_state.js'

const ready = () => [10, 11].map(id => ({ id, online: true, mode: 'test', visionRecent: true }))

test('competition requires both rovers online, in test mode, with recent server frames', () => {
  assert.equal(competitionEligibility(ready()).enabled, true)
  for (const change of [
    rover => { rover.online = false },
    rover => { rover.mode = 'competition' },
    rover => { rover.visionRecent = false },
  ]) {
    const states = ready()
    change(states[1])
    assert.equal(competitionEligibility(states).enabled, false)
    assert.match(competitionEligibility(states).reason, /Rover 11/)
  }
})

test('pose speed handles fields omitted from peer telemetry', () => {
  assert.equal(formatPoseSpeed({ valid: true, col: 2, row: 3 }), 'Velocidad —')
  assert.equal(formatPoseSpeed({ valid: true, speed_cells_s: 1.234 }), 'v 1.23 cel/s')
  assert.equal(formatPoseSpeed({ valid: true, speed_cells_s: 1.234, uncertainty_cells: 0.456 }),
    'v 1.23 cel/s · ±0.46 cel')
})

test('navigation status explains a blocked route and a missing sensor', () => {
  assert.equal(navigationMessage({ navigation: { phase_name: 'blocked', request_id: 7, failure_reason: 1 },
    sensors: { ultrasonic: { valid: true, distance_mm: 500 } } }),
  'Objetivo #7: sin ruta libre al destino; revisa el otro rover y el objetivo')
  assert.equal(targetRejectionMessage('navigation_not_ready', {
    imu: { valid: true, calibrated: false },
  }), 'IMU sin calibrar')
  assert.equal(targetRejectionMessage('peer_no_confirmation'),
    'el compañero no confirmó la orden por ESP-NOW')
  assert.equal(navigationMessage({ navigation: { phase_name: 'driving', has_target: true,
    request_id: 8, motors: { left: 0, right: 0 }, vision: { fresh: true } } }),
  'Objetivo #8: pausado: esperando nueva captura para avance fino')
})

test('diagnostics deduplicate retries and retain 100 lines across boot changes', () => {
  const first = Array.from({ length: 98 }, (_, index) => ({
    boot_id: 1, sequence: index + 1, text: `line ${index + 1}`, uptime_ms: index,
  }))
  let entries = mergeDiagnostics([], first, '2026-10-01T00:00:00.000Z')
  entries = mergeDiagnostics(entries, [first.at(-1),
    { boot_id: 2, sequence: 1, text: 'restarted', uptime_ms: 0 },
    { boot_id: 2, sequence: 2, text: 'ready', uptime_ms: 1 },
  ], '2026-10-01T00:01:00.000Z')
  assert.equal(entries.length, 100)
  assert.equal(entries.at(-2).text, 'restarted')
  assert.equal(mergeDiagnostics(entries, [{ boot_id: 2, sequence: 2, text: 'ready' }], 'later').length, 100)
  entries = mergeDiagnostics(entries, [{ boot_id: 2, sequence: 3, text: 'newest' }], 'later')
  assert.equal(entries.length, 100)
  assert.equal(entries[0].sequence, 2)
  assert.equal(entries.at(-1).text, 'newest')
})
