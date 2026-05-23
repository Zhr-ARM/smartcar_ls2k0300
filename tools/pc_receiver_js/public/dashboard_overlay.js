// dashboard_overlay.js — Canvas 图像叠加层

var showBoundary = true;
var showCenterline = false;
var showAnchor = true;
var showDir = false;
var showSamplePoints = true;

var grayCtx = null;
var ipmCtx = null;

function initOverlayContexts(gCtx, iCtx) {
  grayCtx = gCtx;
  ipmCtx = iCtx;
}

function drawOverlays(status) {
  if (!status) return;
  drawGrayOverlays(status);
  drawIpmOverlays(status);
}

function drawGrayOverlays(status) {
  if (!grayCtx || !grayCtx.canvas) return;
  if (showBoundary) {
    if (Array.isArray(status.left_boundary)) {
      receiverCore.drawPolyline(grayCtx, status.left_boundary, '#ef4444', 1);
    }
    if (Array.isArray(status.right_boundary)) {
      receiverCore.drawPolyline(grayCtx, status.right_boundary, '#3b82f6', 1);
    }
  }
  if (showCenterline) {
    var cl = getSelectedCenterline(status);
    if (cl) receiverCore.drawPolyline(grayCtx, cl, '#22c55e', 1);
  }
}

function drawIpmOverlays(status) {
  if (!ipmCtx || !ipmCtx.canvas) return;
  if (showBoundary) {
    if (Array.isArray(status.left_boundary)) {
      receiverCore.drawPolyline(ipmCtx, status.left_boundary, '#ef4444', 1);
    }
    if (Array.isArray(status.right_boundary)) {
      receiverCore.drawPolyline(ipmCtx, status.right_boundary, '#3b82f6', 1);
    }
  }
  if (showCenterline) {
    var cl = getSelectedCenterline(status);
    if (cl) receiverCore.drawPolyline(ipmCtx, cl, '#22c55e', 1);
  }
  if (showAnchor) {
    var left = Array.isArray(status.cross_lower_left_corner_point) ? status.cross_lower_left_corner_point : null;
    var right = Array.isArray(status.cross_lower_right_corner_point) ? status.cross_lower_right_corner_point : null;
    if (left && left.length === 2) {
      receiverCore.drawPointSet(ipmCtx, [left], '#fbbf24', 4);
    }
    if (right && right.length === 2) {
      receiverCore.drawPointSet(ipmCtx, [right], '#fbbf24', 4);
    }
  }
}

function getSelectedCenterline(status) {
  if (Array.isArray(status.ipm_centerline_selected_shift)) return status.ipm_centerline_selected_shift;
  if (status.ipm_centerline_source === 1) return status.ipm_centerline_from_right_shift;
  return status.ipm_centerline_from_left_shift;
}

function bindOverlayToggles() {
  var el = document.getElementById('togBoundary');
  if (el) el.addEventListener('change', function() { showBoundary = this.checked; });
  el = document.getElementById('togCenterline');
  if (el) el.addEventListener('change', function() { showCenterline = this.checked; });
  el = document.getElementById('togAnchor');
  if (el) el.addEventListener('change', function() { showAnchor = this.checked; });
  el = document.getElementById('togDir');
  if (el) el.addEventListener('change', function() { showDir = this.checked; });
  el = document.getElementById('togSamplePoints');
  if (el) el.addEventListener('change', function() { showSamplePoints = this.checked; });
}
