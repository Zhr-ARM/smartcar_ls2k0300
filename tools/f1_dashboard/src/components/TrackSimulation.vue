<template>
  <div class="h-full w-full relative flex flex-col items-center justify-center bg-slate-900 rounded-xl overflow-hidden shadow-inner border border-slate-700">
    <div class="absolute inset-0 bg-[radial-gradient(ellipse_at_center,rgba(56,189,248,0.1)_0%,transparent_70%)] pointer-events-none"></div>
    <h2 class="absolute top-4 left-4 text-xs font-bold text-sky-400 tracking-[0.2em] font-mono z-20">SIMULATION SANDBOX <br><span class="text-[9px] text-sky-700">TRACK W:45cm | CAR W:18cm</span></h2>
    <div class="absolute top-4 right-4 text-[10px] font-mono text-slate-300 z-20 text-right">
      <div>SRC: {{ data.ipm_centerline_source === 1 ? 'RIGHT_SHIFT' : 'LEFT_SHIFT' }}</div>
      <div>TRACK IDX: {{ data.ipm_track_index ?? -1 }}</div>
    </div>
    
    <!-- Render layer taking exactly proportioned width -->
    <!-- Maximize height, then fix width so that 100% width = 45cm track -->
    <div class="relative h-full flex items-end justify-center w-full max-w-[800px] pb-10 perspective-1000">
      <canvas ref="canvasEl" class="absolute bottom-10 w-full h-[80%] max-h-[600px] z-10" style="filter: drop-shadow(0 0 10px rgba(56,189,248,0.2))"></canvas>
      
      <!-- CSS representation of the car (track:car = 45:18, based on live lane width) -->
      <!-- Center it horizontally -->
      <div
        class="relative z-20 flex justify-center items-center pointer-events-none"
        :style="{ width: `${carRenderWidthPx}px`, transform: `translateX(${carTranslateX}px)` }"
      >
         <!-- Triangular Car Chassis Base -->
         <svg viewBox="0 0 100 120" class="w-full h-auto drop-shadow-2xl opacity-90 overflow-visible">
            <!-- Chassis shape -->
            <polygon points="50,0 10,120 90,120" fill="rgba(15, 23, 42, 0.9)" stroke="#38bdf8" stroke-width="2" />
            <!-- Front roller placeholder -->
            <circle cx="50" cy="15" r="8" fill="#1e293b" stroke="#38bdf8" stroke-width="2" />
         </svg>
         
         <div class="absolute -left-12 bottom-0 flex flex-col items-end gap-1">
           <div class="text-xs font-mono text-cyan-400 drop-shadow">{{ fmtWheel(leftWheelSpeed) }}</div>
             <div class="w-2 h-16 bg-cyan-900/50 rounded-full border border-cyan-800 relative overflow-hidden flex items-end">
             <div class="w-full bg-cyan-400 transition-all duration-75" :style="{ height: wheelFillPct(leftWheelSpeed, data.left_target_count) + '%' }"></div>
             </div>
           <div class="text-[9px] text-slate-500">L {{ wheelDir(leftWheelSpeed) }}</div>
         </div>

         <div class="absolute -right-12 bottom-0 flex flex-col items-start gap-1">
           <div class="text-xs font-mono text-emerald-400 drop-shadow">{{ fmtWheel(rightWheelSpeed) }}</div>
             <div class="w-2 h-16 bg-emerald-900/50 rounded-full border border-emerald-800 relative overflow-hidden flex items-end">
             <div class="w-full bg-emerald-400 transition-all duration-75" :style="{ height: wheelFillPct(rightWheelSpeed, data.right_target_count) + '%' }"></div>
             </div>
           <div class="text-[9px] text-slate-500">R {{ wheelDir(rightWheelSpeed) }}</div>
         </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, computed } from 'vue'

const props = defineProps({
  data: {
    type: Object,
    default: () => ({})
  }
})

const canvasEl = ref(null)
let ctx = null
let cw = 0
let ch = 0

const TRACK_WIDTH_CM = 45
const CAR_WIDTH_CM = 18
const carRenderWidthPx = ref(120)
const carTranslateX = ref(0)

const wheelFillPct = (current, target) => {
  const cur = Number(current || 0)
  const tgt = Number(target || 0)
  const denom = Math.max(1, Math.abs(cur), Math.abs(tgt))
  return Math.min(100, (Math.abs(cur) / denom) * 100)
}

const fmtWheel = (value) => Number(value || 0).toFixed(1)

const wheelDir = (value) => {
  const v = Number(value || 0)
  if (v > 0.001) return 'FWD'
  if (v < -0.001) return 'REV'
  return 'IDLE'
}

const pickWheelSpeed = (current, filtered, target) => {
  const cur = Number(current)
  if (Number.isFinite(cur) && Math.abs(cur) > 0.001) return cur
  const fil = Number(filtered)
  if (Number.isFinite(fil) && Math.abs(fil) > 0.001) return fil
  const tgt = Number(target)
  if (Number.isFinite(tgt)) return tgt
  return 0
}

const leftWheelSpeed = computed(() => pickWheelSpeed(
  props.data.left_current_count,
  props.data.left_filtered_count,
  props.data.left_target_count
))

const rightWheelSpeed = computed(() => pickWheelSpeed(
  props.data.right_current_count,
  props.data.right_filtered_count,
  props.data.right_target_count
))

onMounted(() => {
  if (canvasEl.value) {
    // Make canvas high-res based on its client rect
    const rect = canvasEl.value.getBoundingClientRect()
    cw = rect.width * window.devicePixelRatio
    ch = rect.height * window.devicePixelRatio
    canvasEl.value.width = cw
    canvasEl.value.height = ch
    ctx = canvasEl.value.getContext('2d')
    ctx.scale(window.devicePixelRatio, window.devicePixelRatio)
    cw = rect.width
    ch = rect.height
    
    drawSandbox()
  }
})

const drawSandbox = () => {
    if (!ctx) return
    ctx.clearRect(0, 0, cw, ch)

    const { 
        ipm_left_boundary = [], 
        ipm_right_boundary = [],
      ipm_left_boundary_corner = [],
      ipm_right_boundary_corner = [],
      ipm_centerline_selected_shift = [],
        ipm_track_point = [0,0],
      ipm_track_valid = false,
      ipm_weighted_decision_point = [0, 0],
      ipm_centerline_source = 0,
      ipm_size = [188, 240]
    } = props.data

    const normalizePoints = (raw) => {
      if (!Array.isArray(raw) || raw.length === 0) return []
      if (Array.isArray(raw[0])) {
        return raw
          .filter((p) => Array.isArray(p) && p.length >= 2)
          .map((p) => [Number(p[0]), Number(p[1])])
          .filter((p) => Number.isFinite(p[0]) && Number.isFinite(p[1]))
      }
      const pts = []
      for (let i = 0; i < raw.length; i += 2) {
        const x = Number(raw[i])
        const y = Number(raw[i + 1])
        if (Number.isFinite(x) && Number.isFinite(y)) pts.push([x, y])
      }
      return pts
    }

    const pickBottomPoint = (pts) => {
      if (!Array.isArray(pts) || pts.length === 0) return null
      let best = pts[0]
      for (let i = 1; i < pts.length; i += 1) {
        if (pts[i][1] > best[1]) best = pts[i]
      }
      return best
    }

    // We assume IPM x range spans roughly the track width 45cm. 
    // Usually IPM x=0 is left, x=width is right. Let's say IPM width is 188 as before.
    // The canvas represents the IPM bounding box directly.
    const ipmW = Array.isArray(ipm_size) && ipm_size[0] ? ipm_size[0] : 188
    const ipmH = Array.isArray(ipm_size) && ipm_size[1] ? ipm_size[1] : 240
    
    // Helper to map IPM to Canvas Size
    const mapX = (x) => (x / ipmW) * cw
    // Keep the same top-left origin convention as pc_receiver_js drawSeries.
    const mapY = (y) => (y / ipmH) * ch

    const drawPoints = (points, color, lineWidth=3) => {
        const pts = normalizePoints(points)
        if(pts.length < 2) return
        ctx.beginPath()
        ctx.strokeStyle = color
        ctx.lineWidth = lineWidth
        ctx.lineJoin = 'round'
        ctx.lineCap = 'round'
        for (let i = 0; i < pts.length; i += 1) {
            const px = mapX(pts[i][0])
            const py = mapY(pts[i][1])
            if (i===0) ctx.moveTo(px, py)
            else ctx.lineTo(px, py)
        }
        ctx.stroke()
    }

    const drawDots = (points, color, size = 2) => {
      const pts = normalizePoints(points)
      if (pts.length < 1) return
      ctx.fillStyle = color
      for (let i = 0; i < pts.length; i += 1) {
        const px = mapX(pts[i][0])
        const py = mapY(pts[i][1])
        ctx.beginPath()
        ctx.arc(px, py, size, 0, Math.PI * 2)
        ctx.fill()
      }
    }

    const drawTrackSurface = (leftRaw, rightRaw) => {
      const leftPts = normalizePoints(leftRaw)
      const rightPts = normalizePoints(rightRaw)
      const n = Math.min(leftPts.length, rightPts.length)
      if (n < 2) return
      ctx.beginPath()
      for (let i = 0; i < n; i += 1) {
        const px = mapX(leftPts[i][0])
        const py = mapY(leftPts[i][1])
        if (i === 0) ctx.moveTo(px, py)
        else ctx.lineTo(px, py)
      }
      for (let i = n - 1; i >= 0; i -= 1) {
        ctx.lineTo(mapX(rightPts[i][0]), mapY(rightPts[i][1]))
      }
      ctx.closePath()
      const grd = ctx.createLinearGradient(0, 0, 0, ch)
      grd.addColorStop(0, 'rgba(56, 189, 248, 0.08)')
      grd.addColorStop(1, 'rgba(15, 23, 42, 0.22)')
      ctx.fillStyle = grd
      ctx.fill()
    }

    const leftPts = normalizePoints(ipm_left_boundary)
    const rightPts = normalizePoints(ipm_right_boundary)
    const centerPts = normalizePoints(ipm_centerline_selected_shift)
    const hasTrackData = leftPts.length > 1 || rightPts.length > 1 || centerPts.length > 1

    if (leftPts.length > 0 && rightPts.length > 0) {
      const leftBottom = pickBottomPoint(leftPts)
      const rightBottom = pickBottomPoint(rightPts)
      if (leftBottom && rightBottom) {
        const laneWidthPx = Math.abs(mapX(rightBottom[0]) - mapX(leftBottom[0]))
        const laneCenterX = (mapX(rightBottom[0]) + mapX(leftBottom[0])) * 0.5
        const carW = laneWidthPx * (CAR_WIDTH_CM / TRACK_WIDTH_CM)
        carRenderWidthPx.value = Math.max(24, Math.min(cw * 0.5, carW))
        carTranslateX.value = laneCenterX - (cw * 0.5)
      }
    }

    if (hasTrackData) {
      drawTrackSurface(leftPts, rightPts)
    }

    // Draw Boundaries (Thick Neon Lines)
    drawPoints(leftPts, 'rgba(56, 189, 248, 0.9)', 4) // Sky blue
    drawPoints(rightPts, 'rgba(244, 63, 94, 0.9)', 4) // Rose

    // Draw corners for boundary trend only.
    drawDots(ipm_left_boundary_corner, 'rgba(14, 165, 233, 0.9)', 2.5)
    drawDots(ipm_right_boundary_corner, 'rgba(244, 63, 94, 0.9)', 2.5)

    // Draw Centerline
    if (centerPts.length > 0) {
        ctx.setLineDash([10, 10])
      drawPoints(centerPts, 'rgba(250, 204, 21, 0.9)', 2) // Yellow dashed
        ctx.setLineDash([])
    }

    // Draw Track Point (Red Crosshair)
    if (ipm_track_valid) {
        const tx = mapX(ipm_track_point[0])
        const ty = mapY(ipm_track_point[1])
        
        ctx.beginPath()
        ctx.arc(tx, ty, 8, 0, Math.PI * 2)
        ctx.strokeStyle = '#ef4444' // Red
        ctx.lineWidth = 3
        ctx.stroke()
        
        ctx.beginPath()
        ctx.moveTo(tx - 15, ty)
        ctx.lineTo(tx + 15, ty)
        ctx.moveTo(tx, ty - 15)
        ctx.lineTo(tx, ty + 15)
        ctx.strokeStyle = '#fca5a5'
        ctx.lineWidth = 2
        ctx.stroke()
        
        ctx.fillStyle = 'rgba(239, 68, 68, 0.2)'
        ctx.fill()
    }

    if (ipm_weighted_decision_point && ipm_weighted_decision_point.length >= 2) {
      const dx = mapX(ipm_weighted_decision_point[0])
      const dy = mapY(ipm_weighted_decision_point[1])
      ctx.beginPath()
      ctx.arc(dx, dy, 6, 0, Math.PI * 2)
      ctx.fillStyle = 'rgba(34, 197, 94, 0.25)'
      ctx.fill()
      ctx.beginPath()
      ctx.arc(dx, dy, 6, 0, Math.PI * 2)
      ctx.strokeStyle = '#4ade80'
      ctx.lineWidth = 2
      ctx.stroke()
    }

    ctx.fillStyle = 'rgba(148, 163, 184, 0.9)'
    ctx.font = '11px ui-monospace, SFMono-Regular, Menlo, monospace'
    ctx.fillText(`source: ${ipm_centerline_source === 1 ? 'right_shift' : 'left_shift'}`, 12, 18)

    if (!hasTrackData) {
      ctx.fillStyle = 'rgba(100, 116, 139, 0.95)'
      ctx.font = '12px ui-monospace, SFMono-Regular, Menlo, monospace'
      ctx.fillText('waiting ipm track data...', 12, 40)
    }

    requestAnimationFrame(drawSandbox)
}
</script>

<style scoped>
.perspective-1000 {
  perspective: 1000px;
}
</style>
