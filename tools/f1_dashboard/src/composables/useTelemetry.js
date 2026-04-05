import { reactive } from 'vue'
import { useWebSocket } from '@vueuse/core'

export function useTelemetry(wsUrl = 'ws://localhost:8080') {
  const currentStatus = reactive({
    message: 'disconnected',
    message_seq: 0,
    base_speed: 0,
    left_target_count: 0,
    right_target_count: 0,
    left_current_count: 0,
    right_current_count: 0,
    left_duty: 0,
    right_duty: 0,
    line_error: 0,
    otsu_threshold: 0,
    web_data_profile: 0,
    capture_wait_us: 0,
    preprocess_us: 0,
    otsu_us: 0,
    maze_us: 0,
    total_us: 0,
    
    // Arrays
    left_boundary: [],
    right_boundary: [],
    ipm_left_boundary: [],
    ipm_right_boundary: [],
    ipm_left_boundary_corner: [],
    ipm_right_boundary_corner: [],
    left_auxiliary_line: [],
    right_auxiliary_line: [],
    left_auxiliary_seed: [],
    right_auxiliary_seed: [],
    left_boundary_corner: [],
    right_boundary_corner: [],
    ipm_centerline_selected_shift: [],
    src_centerline_selected_shift: [],
    
    // Complex features
    ipm_left_boundary_curvature: [],
    ipm_right_boundary_curvature: [],
    ipm_left_boundary_angle_cos: [],
    ipm_right_boundary_angle_cos: [],
    ipm_centerline_selected_curvature: [],
    ipm_centerline_selected_count: 0,
    src_centerline_selected_count: 0,
    gray_size: [188, 120],
    ipm_size: [188, 240],
    
    roi: [],
    red: [],
    roi_valid: false,
    red_found: false,
    ipm_track_valid: false,
    ipm_track_method: 0,
    ipm_centerline_source: 0,
    ipm_track_index: -1,
    ipm_track_point: [0, 0],
    ipm_weighted_first_point_error: 0,
    ipm_weighted_decision_point: [0, 0],
    src_weighted_decision_point: [0, 0]
  })

  const currentFrame = reactive({
    width: 0,
    height: 0,
    mode: 1, 
    blobUrl: null
  })

  const linkStats = reactive({
    status_fps: 0,
    frame_fps: 0,
    last_status_ts: 0,
    last_frame_ts: 0,
    total_status: 0,
    total_frame: 0
  })

  let statusHits = 0
  let frameHits = 0
  setInterval(() => {
    linkStats.status_fps = statusHits
    linkStats.frame_fps = frameHits
    statusHits = 0
    frameHits = 0
  }, 1000)

  const { status } = useWebSocket(wsUrl, {
    autoReconnect: {
      retries: () => true,
      delay: 500,
    },
    onMessage: (ws, event) => {
      try {
        const msg = JSON.parse(event.data)
        if (msg.type === 'status') {
          statusHits += 1
          linkStats.total_status += 1
          linkStats.last_status_ts = Date.now()
          Object.assign(currentStatus, msg.data, {
            message_seq: currentStatus.message_seq + 1
          })
        } else if (msg.type === 'frame') {
          frameHits += 1
          linkStats.total_frame += 1
          linkStats.last_frame_ts = Date.now()
          currentFrame.width = msg.data.width
          currentFrame.height = msg.data.height
          currentFrame.mode = msg.data.mode
        }
      } catch (err) {
        console.error('WS Parse Error', err)
      }
    }
  })

  const getImageUrl = (mode) => {
    let modeStr = 'gray'
    if (mode === 0) modeStr = 'binary'
    else if (mode === 2) modeStr = 'rgb'
    const safeWs = wsUrl || 'ws://127.0.0.1:8080'
    const url = new URL(safeWs)
    url.protocol = 'http:'
    return `${url.origin}/api/frame_${modeStr}.jpg?t=${Date.now()}`
  }

  return {
    wsStatus: status,
    telemetry: currentStatus,
    currentFrame,
    linkStats,
    getImageUrl
  }
}
