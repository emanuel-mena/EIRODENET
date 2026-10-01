export function competitionEligibility(states) {
  for (const rover of states) {
    if (!rover.online) return { enabled: false, reason: `Rover ${rover.id} sin conexión` }
  }
  for (const rover of states) {
    if (rover.mode !== 'test') return {
      enabled: false,
      reason: `Rover ${rover.id} ya está en competencia; usa BOOT para volver a prueba`,
    }
  }
  for (const rover of states) {
    if (!rover.visionRecent) return {
      enabled: false,
      reason: `Rover ${rover.id} no recibe tramas válidas recientes del servidor`,
    }
  }
  return { enabled: true, reason: 'Ambos rovers listos · datos del servidor recientes' }
}

export function mergeDiagnostics(previous, incoming, receivedAt, max = 100) {
  const entries = Array.isArray(previous) ? previous.slice(-max) : []
  const seen = new Set(entries.map(item => `${item.boot_id}:${item.sequence}`))
  for (const item of incoming || []) {
    if (!Number.isInteger(item.boot_id) || !Number.isInteger(item.sequence) ||
        typeof item.text !== 'string') continue
    const key = `${item.boot_id}:${item.sequence}`
    if (seen.has(key)) continue
    seen.add(key)
    entries.push({ boot_id: item.boot_id, sequence: item.sequence,
      uptime_ms: item.uptime_ms, text: item.text, receivedAt })
  }
  return entries.slice(-max)
}
