<template>
  <div class="h-screen w-screen bg-slate-900 text-slate-100 flex flex-col p-4 box-border overflow-hidden antialiased font-sans">
    <header class="flex justify-between items-center bg-slate-800 p-4 rounded-xl shadow-lg border border-slate-700 mb-4 z-10 relative">
      <div class="flex items-center gap-4">
        <h1 class="text-2xl font-bold bg-clip-text text-transparent bg-gradient-to-r from-sky-400 to-blue-600">LoongCar F1 Dashboard</h1>
        <span class="px-3 py-1 text-xs font-bold rounded-full border" 
             :class="wsStatus === 'OPEN' ? 'border-sky-500/50 text-sky-400 bg-sky-500/10 shadow-[0_0_10px_rgba(56,189,248,0.3)]' : 'border-rose-500/50 text-rose-400 bg-rose-500/10'">
          {{ wsStatus === 'OPEN' ? 'LIVE DATA STREAM' : 'NO SIGNAL' }}
        </span>
      </div>
      <div class="flex gap-2">
        <button @click="connectHost('ws://127.0.0.1:8080')" class="px-4 py-2 bg-slate-700 hover:bg-slate-600 rounded drop-shadow transition-colors font-medium text-sm text-sky-50">主后端 (8080)</button>
        <button @click="connectHost('ws://127.0.0.1:8180')" class="px-4 py-2 bg-slate-700 hover:bg-slate-600 rounded drop-shadow transition-colors font-medium text-sm text-sky-50">本地计算服 (8180)</button>
      </div>
    </header>

    <main class="flex-1 grid grid-rows-[minmax(0,1fr)_300px] gap-4 relative h-full min-h-0">
      <section class="grid grid-cols-12 gap-4 min-h-0">
      <!-- LEFT PANEL: Motor and PID Telemetry -->
      <div class="col-span-3 flex flex-col h-full gap-4 overflow-auto pr-1">
        <div class="h-56 bg-slate-800 rounded-xl p-4 border border-slate-700 shadow-xl flex flex-col min-h-0">
          <h2 class="text-xs font-bold text-slate-400 mb-2 uppercase tracking-widest flex justify-between">
            <span>MOTOR PID TRACKING</span>
            <span class="text-[9px] text-sky-500">Cur vs Target</span>
          </h2>
          <!-- Reusing TelemetryCharts but passing specific fields -->
          <TelemetryCharts :data="telemetry" chartType="motor" />
        </div>

        <div class="h-56 bg-slate-800 rounded-xl p-4 border border-slate-700 shadow-xl flex flex-col min-h-0">
          <h2 class="text-xs font-bold text-slate-400 mb-2 uppercase tracking-widest flex justify-between">
            <span>DUTY OUTPUT</span>
            <span class="text-[9px] text-cyan-400">duty/hw duty</span>
          </h2>
          <TelemetryCharts :data="telemetry" chartType="duty" />
        </div>

        <div class="h-56 bg-slate-800 rounded-xl p-4 border border-slate-700 shadow-xl flex flex-col min-h-0">
          <h2 class="text-xs font-bold text-slate-400 mb-2 uppercase tracking-widest flex justify-between">
            <span>LINE ERROR</span>
            <span class="text-[9px] text-rose-400">error & base_speed</span>
          </h2>
          <TelemetryCharts :data="telemetry" chartType="error" />
        </div>

        <div class="h-56 bg-slate-800 rounded-xl p-4 border border-slate-700 shadow-xl flex flex-col min-h-0">
          <h2 class="text-xs font-bold text-slate-400 mb-2 uppercase tracking-widest flex justify-between">
            <span>CENTERLINE CURVATURE</span>
            <span class="text-[9px] text-amber-400">ipm_curvature</span>
          </h2>
          <TelemetryCharts :data="telemetry" chartType="curvature" />
        </div>
      </div>
      
      <!-- CENTER PANEL: 3D-like Track Sandbox -->
      <div class="col-span-6 flex flex-col h-full gap-2">
         <div class="px-1 flex justify-between items-center text-[10px] font-mono text-slate-400">
           <span>TRACK SANDBOX</span>
           <span :class="telemetry.roi_valid ? 'text-cyan-300' : 'text-slate-500'">
             ROI {{ telemetry.roi_valid ? 'ACTIVE' : 'OFF' }} | RED {{ Array.isArray(telemetry.red) ? telemetry.red.length : 0 }}
           </span>
         </div>
         <TrackSimulation :data="telemetry" />
      </div>

      <!-- RIGHT PANEL: Vision Geometry and Camera -->
      <div class="col-span-3 flex flex-col h-full gap-4 overflow-auto pr-1">
        <div class="h-52 bg-slate-800 rounded-xl p-4 border border-slate-700 shadow-xl flex flex-col min-h-0">
          <h2 class="text-xs font-bold text-slate-400 mb-2 uppercase tracking-widest flex justify-between">
            <span>BOUNDARY ANGLES (COS)</span>
            <span class="text-[9px] text-rose-400">left vs right cos</span>
          </h2>
          <TelemetryCharts :data="telemetry" chartType="angle" />
        </div>

        <div class="h-52 bg-slate-800 rounded-xl p-4 border border-slate-700 shadow-xl flex flex-col min-h-0">
          <h2 class="text-xs font-bold text-slate-400 mb-2 uppercase tracking-widest flex justify-between">
            <span>PERF COST (US)</span>
            <span class="text-[9px] text-orange-400">capture/pre/otsu/maze</span>
          </h2>
          <TelemetryCharts :data="telemetry" chartType="perf" />
        </div>

        <div class="h-52 bg-slate-800 rounded-xl p-4 border border-slate-700 shadow-xl flex flex-col min-h-0">
          <h2 class="text-xs font-bold text-slate-400 mb-2 uppercase tracking-widest flex justify-between">
            <span>POINT COUNTS</span>
            <span class="text-[9px] text-lime-400">maze & centerline</span>
          </h2>
          <TelemetryCharts :data="telemetry" chartType="points" />
        </div>

        <!-- Camera & Error Metrics -->
        <div class="h-52 bg-slate-800 rounded-xl border border-slate-700 shadow-xl flex flex-col overflow-hidden">
          <div class="flex justify-between items-center p-3 border-b border-slate-700 bg-slate-800/80">
            <h2 class="text-xs font-bold text-slate-400 uppercase tracking-widest">LIVE CAMERA</h2>
            <div class="text-[10px] font-mono" :class="Math.abs(telemetry.line_error) > 40 ? 'text-rose-400' : 'text-emerald-400'">
              ERR: {{ telemetry.line_error?.toFixed(1) || 0 }}
            </div>
          </div>
          <div class="flex-1 bg-black relative flex items-center justify-center overflow-hidden">
            <span v-if="!currentFrame.width" class="text-slate-700 font-mono absolute z-0 text-sm">NO IMAGE FEED</span>
            <img v-else :src="imageUrl" class="max-w-full max-h-full object-contain z-10" />
            <!-- Camera Stats Overlay -->
            <div class="absolute bottom-2 right-2 text-[9px] font-mono text-white/50 bg-black/50 px-1 rounded z-20">
               {{ telemetry.total_us ? (telemetry.total_us/1000).toFixed(1)+'ms Algo Latency' : '--' }}
            </div>
          </div>
        </div>
      </div>
      </section>

      <section class="min-h-0">
        <TelemetryInspector :data="telemetry" :linkStats="linkStats" />
      </section>
    </main>
  </div>
</template>

<script setup>
import { ref, onMounted } from 'vue'
import { useTelemetry } from './composables/useTelemetry'
import TelemetryCharts from './components/TelemetryCharts.vue'
import TrackSimulation from './components/TrackSimulation.vue'
import TelemetryInspector from './components/TelemetryInspector.vue'

const hostFromQuery = new URLSearchParams(window.location.search).get('host')
const currentHost = ref(hostFromQuery || 'ws://127.0.0.1:8080')
const { wsStatus, telemetry, currentFrame, linkStats, getImageUrl } = useTelemetry(currentHost.value)
const imageUrl = ref('')

const connectHost = (url) => {
  window.location.search = `?host=${encodeURIComponent(url)}`
}

onMounted(() => {
  setInterval(() => {
    if (wsStatus.value === 'OPEN' && currentFrame.width > 0) {
      imageUrl.value = getImageUrl(currentFrame.mode)
    }
  }, 100)
})
</script>
