import './style.css'

document.querySelector('#app').innerHTML = `
  <section class="shell">
    <div class="eyebrow"><span></span> EIRODENET · LOCAL SITE</div>
    <h1>Rover <em>en línea.</em></h1>
    <p class="lead">Esta página se sirve directamente desde el sistema de archivos del ESP32.</p>
    <div class="grid">
      <article>
        <small>ORIGEN</small>
        <strong>SPIFFS</strong>
        <p>Contenido estático compilado con Vite y almacenado en la flash.</p>
      </article>
      <article>
        <small>DIRECCIÓN LOCAL</small>
        <strong>${window.location.hostname || 'nombre.local'}</strong>
        <p>Descubierta en la red mediante mDNS.</p>
      </article>
      <article>
        <small>ESTADO</small>
        <strong class="ok">DISPONIBLE</strong>
        <p>${new Date().toLocaleString('es-CR')}</p>
      </article>
    </div>
  </section>
`
