<template>
  <div class="h-full w-full flex flex-col relative overflow-hidden text-white/80" ref="chartContainer">
    <div class="flex-1 min-h-0" ref="chartEl"></div>
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted, watch } from 'vue'
import * as echarts from 'echarts'

const props = defineProps({
  data: { type: Object, default: () => ({}) },
  chartType: { type: String, default: 'motor' } // motor, curvature, angle, duty, perf, error, points
})

const chartEl = ref(null)
let chartInstance = null

const historyCount = 200
const buffer = { axis: [], y1: [], y2: [], y3: [], y4: [] }

const pointCount = (arr) => {
  if (!Array.isArray(arr) || arr.length === 0) return 0
  if (Array.isArray(arr[0])) return arr.length
  return Math.floor(arr.length / 2)
}

onMounted(() => {
  chartInstance = echarts.init(chartEl.value, 'dark')
  
  let seriesConfigs = []
  if(props.chartType === 'motor') {
    seriesConfigs = [
      { name: 'Left Cur', type: 'line', showSymbol: false, itemStyle: { color: '#06b6d4' } },
      { name: 'Left Tg', type: 'line', lineStyle: { type: 'dashed' }, showSymbol: false, itemStyle: { color: '#0891b2' } },
      { name: 'Right Cur', type: 'line', showSymbol: false, itemStyle: { color: '#10b981' } },
      { name: 'Right Tg', type: 'line', lineStyle: { type: 'dashed' }, showSymbol: false, itemStyle: { color: '#059669' } }
    ]
  } else if (props.chartType === 'curvature') {
    seriesConfigs = [
      { name: 'Center Curv', type: 'line', showSymbol: false, areaStyle: { color: 'rgba(250, 204, 21, 0.2)' }, itemStyle: { color: '#facc15' } }
    ]
  } else if (props.chartType === 'angle') {
    seriesConfigs = [
      { name: 'Left Cos', type: 'scatter', symbolSize: 3, itemStyle: { color: '#38bdf8' } },
      { name: 'Right Cos', type: 'scatter', symbolSize: 3, itemStyle: { color: '#fb7185' } }
    ]
  } else if (props.chartType === 'duty') {
    seriesConfigs = [
      { name: 'Left Duty', type: 'line', showSymbol: false, itemStyle: { color: '#06b6d4' } },
      { name: 'Right Duty', type: 'line', showSymbol: false, itemStyle: { color: '#34d399' } },
      { name: 'L HW Duty', type: 'line', showSymbol: false, lineStyle: { type: 'dashed' }, itemStyle: { color: '#0ea5e9' } },
      { name: 'R HW Duty', type: 'line', showSymbol: false, lineStyle: { type: 'dashed' }, itemStyle: { color: '#10b981' } }
    ]
  } else if (props.chartType === 'perf') {
    seriesConfigs = [
      { name: 'capture', type: 'line', showSymbol: false, itemStyle: { color: '#38bdf8' } },
      { name: 'pre', type: 'line', showSymbol: false, itemStyle: { color: '#22d3ee' } },
      { name: 'otsu', type: 'line', showSymbol: false, itemStyle: { color: '#facc15' } },
      { name: 'maze', type: 'line', showSymbol: false, itemStyle: { color: '#fb7185' } },
      { name: 'total', type: 'line', showSymbol: false, lineStyle: { width: 2 }, itemStyle: { color: '#f97316' } }
    ]
  } else if (props.chartType === 'error') {
    seriesConfigs = [
      { name: 'line_error', type: 'line', showSymbol: false, itemStyle: { color: '#f43f5e' } },
      { name: 'weighted_error', type: 'line', showSymbol: false, itemStyle: { color: '#f59e0b' } },
      { name: 'base_speed', type: 'line', showSymbol: false, itemStyle: { color: '#60a5fa' } }
    ]
  } else if (props.chartType === 'points') {
    seriesConfigs = [
      { name: 'maze_left', type: 'line', showSymbol: false, itemStyle: { color: '#06b6d4' } },
      { name: 'maze_right', type: 'line', showSymbol: false, itemStyle: { color: '#10b981' } },
      { name: 'center_count', type: 'line', showSymbol: false, itemStyle: { color: '#eab308' } }
    ]
  }

  const option = {
    backgroundColor: 'transparent',
    tooltip: { trigger: 'axis', animation: false, textStyle: { fontSize: 10 } },
    grid: { left: 40, right: 10, top: 10, bottom: 20 },
    animation: false,
    xAxis: {
      type: 'category',
      splitLine: { show: true, lineStyle: { color: 'rgba(255,255,255,0.05)' } },
      axisLabel: { show: false }
    },
    yAxis: {
      type: 'value',
      splitLine: { show: true, lineStyle: { color: 'rgba(255,255,255,0.05)' } }
    },
    series: seriesConfigs.map(s => ({ ...s, data: [] }))
  }

  chartInstance.setOption(option)
  
  const resizeObserver = new ResizeObserver(() => {
    chartInstance.resize()
  })
  resizeObserver.observe(chartEl.value)
})

onUnmounted(() => {
  if (chartInstance) chartInstance.dispose()
})

const processArrayForChart = (arr) => {
  // If array contains points/curvature like [x0, y0, k0, x1, y1, k1...] we just take an average or max for plotting a single timeline value.
  // Assuming it's simple array of floats:
  if (!arr || arr.length === 0) return 0
  let sum = 0
  for(let i=0; i<arr.length; i++){
     sum += arr[i]
  }
  return sum / arr.length
}

watch(() => props.data?.message_seq, () => {
    if(!chartInstance) return;
    const nowStr = new Date().toISOString()
    buffer.axis.push(nowStr)

    if (props.chartType === 'motor') {
      buffer.y1.push(props.data.left_current_count || 0)
      buffer.y2.push(props.data.left_target_count || 0)
      buffer.y3.push(props.data.right_current_count || 0)
      buffer.y4.push(props.data.right_target_count || 0)
    } else if (props.chartType === 'curvature') {
      const centerCurvAvg = processArrayForChart(props.data.ipm_centerline_selected_curvature)
      buffer.y1.push(centerCurvAvg)
    } else if (props.chartType === 'angle') {
      buffer.y1.push(processArrayForChart(props.data.ipm_left_boundary_angle_cos))
      buffer.y2.push(processArrayForChart(props.data.ipm_right_boundary_angle_cos))
    } else if (props.chartType === 'duty') {
      buffer.y1.push(props.data.left_duty || 0)
      buffer.y2.push(props.data.right_duty || 0)
      buffer.y3.push(props.data.left_hardware_duty || 0)
      buffer.y4.push(props.data.right_hardware_duty || 0)
    } else if (props.chartType === 'perf') {
      buffer.y1.push(props.data.capture_wait_us || 0)
      buffer.y2.push(props.data.preprocess_us || 0)
      buffer.y3.push(props.data.otsu_us || 0)
      buffer.y4.push(props.data.maze_us || 0)
      if (!buffer.y5) buffer.y5 = []
      buffer.y5.push(props.data.total_us || 0)
    } else if (props.chartType === 'error') {
      buffer.y1.push(props.data.line_error || 0)
      buffer.y2.push(props.data.ipm_weighted_first_point_error || 0)
      buffer.y3.push(props.data.base_speed || 0)
    } else if (props.chartType === 'points') {
      buffer.y1.push(pointCount(props.data.ipm_left_boundary))
      buffer.y2.push(pointCount(props.data.ipm_right_boundary))
      buffer.y3.push(pointCount(props.data.ipm_centerline_selected_shift))
    }

    if (buffer.axis.length > historyCount) {
        buffer.axis.shift()
        buffer.y1.shift()
        if (props.chartType === 'motor' || props.chartType === 'angle' || props.chartType === 'duty' || props.chartType === 'perf' || props.chartType === 'error' || props.chartType === 'points') buffer.y2.shift()
        if (props.chartType === 'motor' || props.chartType === 'duty' || props.chartType === 'perf' || props.chartType === 'error' || props.chartType === 'points') buffer.y3.shift()
        if (props.chartType === 'motor' || props.chartType === 'duty' || props.chartType === 'perf') buffer.y4.shift()
        if (props.chartType === 'perf' && buffer.y5) buffer.y5.shift()
    }
    
    const seriesData = [{ data: buffer.y1 }]
    if (props.chartType === 'motor') {
        seriesData.push({ data: buffer.y2 }, { data: buffer.y3 }, { data: buffer.y4 })
    } else if (props.chartType === 'angle') {
        seriesData.push({ data: buffer.y2 })
    } else if (props.chartType === 'duty') {
      seriesData.push({ data: buffer.y2 }, { data: buffer.y3 }, { data: buffer.y4 })
    } else if (props.chartType === 'perf') {
      seriesData.push({ data: buffer.y2 }, { data: buffer.y3 }, { data: buffer.y4 }, { data: buffer.y5 || [] })
    } else if (props.chartType === 'error' || props.chartType === 'points') {
      seriesData.push({ data: buffer.y2 }, { data: buffer.y3 })
    }

    chartInstance.setOption({
        xAxis: { data: buffer.axis },
        series: seriesData
    })
})
</script>
