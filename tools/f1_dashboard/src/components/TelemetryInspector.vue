<template>
  <div class="bg-slate-800 rounded-xl border border-slate-700 shadow-xl p-3 h-full min-h-0 flex flex-col">
    <div class="flex items-center justify-between mb-2">
      <h3 class="text-xs font-bold tracking-[0.2em] text-sky-300">LINK HEALTH</h3>
      <span class="text-[10px] font-mono text-slate-400">{{ formatTime(linkStats.last_status_ts) }}</span>
    </div>

    <div class="grid grid-cols-6 gap-2 text-[11px] font-mono mb-3">
      <div class="bg-slate-900/80 rounded p-2 border border-slate-700">
        <div class="text-slate-400">STATUS FPS</div>
        <div class="text-sky-300 text-base">{{ linkStats.status_fps }}</div>
      </div>
      <div class="bg-slate-900/80 rounded p-2 border border-slate-700">
        <div class="text-slate-400">FRAME FPS</div>
        <div class="text-emerald-300 text-base">{{ linkStats.frame_fps }}</div>
      </div>
      <div class="bg-slate-900/80 rounded p-2 border border-slate-700">
        <div class="text-slate-400">TOTAL STATUS</div>
        <div class="text-slate-100 text-base">{{ linkStats.total_status }}</div>
      </div>
      <div class="bg-slate-900/80 rounded p-2 border border-slate-700">
        <div class="text-slate-400">TOTAL FRAME</div>
        <div class="text-slate-100 text-base">{{ linkStats.total_frame }}</div>
      </div>
      <div class="bg-slate-900/80 rounded p-2 border border-slate-700">
        <div class="text-slate-400">GRAY SIZE</div>
        <div class="text-slate-100 text-base">{{ fmtSize(data.gray_size) }}</div>
      </div>
      <div class="bg-slate-900/80 rounded p-2 border border-slate-700">
        <div class="text-slate-400">IPM SIZE</div>
        <div class="text-slate-100 text-base">{{ fmtSize(data.ipm_size) }}</div>
      </div>
    </div>

    <div class="grid grid-cols-4 gap-2 text-[10px] font-mono mb-3">
      <div v-for="item in keyStats" :key="item.label" class="bg-slate-900/60 border border-slate-700 rounded p-2">
        <div class="text-slate-500 truncate">{{ item.label }}</div>
        <div class="text-slate-100">{{ item.value }}</div>
      </div>
    </div>

    <div class="flex-1 min-h-0 overflow-auto rounded border border-slate-700 bg-slate-900/70 p-2">
      <table class="w-full text-[10px] font-mono">
        <thead>
          <tr class="text-slate-400 border-b border-slate-700">
            <th class="text-left py-1 pr-2">FIELD</th>
            <th class="text-left py-1 pr-2">TYPE</th>
            <th class="text-left py-1">VALUE / COUNT</th>
          </tr>
        </thead>
        <tbody>
          <tr v-for="row in rows" :key="row.name" class="border-b border-slate-800/60 text-slate-200">
            <td class="py-1 pr-2">{{ row.name }}</td>
            <td class="py-1 pr-2 text-slate-400">{{ row.type }}</td>
            <td class="py-1">{{ row.value }}</td>
          </tr>
        </tbody>
      </table>
    </div>
  </div>
</template>

<script setup>
import { computed } from 'vue'

const props = defineProps({
  data: { type: Object, default: () => ({}) },
  linkStats: { type: Object, default: () => ({}) }
})

const fmtSize = (arr) => {
  if (!Array.isArray(arr) || arr.length < 2) return '--'
  return `${arr[0]}x${arr[1]}`
}

const formatTime = (ts) => {
  if (!ts) return '--'
  return new Date(ts).toLocaleTimeString()
}

const keyStats = computed(() => [
  { label: 'ts_ms', value: String(props.data.ts_ms ?? '--') },
  { label: 'total_us', value: Number(props.data.total_us || 0).toFixed(0) },
  { label: 'line_error', value: Number(props.data.line_error || 0).toFixed(2) },
  { label: 'base_speed', value: Number(props.data.base_speed || 0).toFixed(2) },
  { label: 'left_current_count', value: Number(props.data.left_current_count || 0).toFixed(2) },
  { label: 'right_current_count', value: Number(props.data.right_current_count || 0).toFixed(2) },
  { label: 'left_target_count', value: Number(props.data.left_target_count || 0).toFixed(2) },
  { label: 'right_target_count', value: Number(props.data.right_target_count || 0).toFixed(2) },
  { label: 'otsu_threshold', value: Number(props.data.otsu_threshold || 0).toFixed(2) },
  { label: 'ipm_track_valid', value: props.data.ipm_track_valid ? 'true' : 'false' }
])

const rows = computed(() => {
  const fields = [
    'left_boundary',
    'right_boundary',
    'left_auxiliary_line',
    'right_auxiliary_line',
    'left_boundary_corner',
    'right_boundary_corner',
    'ipm_left_boundary',
    'ipm_right_boundary',
    'ipm_left_boundary_corner',
    'ipm_right_boundary_corner',
    'ipm_centerline_selected_shift',
    'src_centerline_selected_shift',
    'ipm_centerline_selected_curvature',
    'ipm_left_boundary_curvature',
    'ipm_right_boundary_curvature',
    'ipm_left_boundary_angle_cos',
    'ipm_right_boundary_angle_cos',
    'ipm_track_point',
    'roi',
    'red'
  ]
  return fields.map((name) => {
    const value = props.data[name]
    if (Array.isArray(value)) {
      return {
        name,
        type: 'array',
        value: `count=${value.length}`
      }
    }
    return {
      name,
      type: typeof value,
      value: value === undefined ? '--' : String(value)
    }
  })
})
</script>
