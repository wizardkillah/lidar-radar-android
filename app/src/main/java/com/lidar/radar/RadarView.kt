package com.lidar.radar

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.RectF
import android.view.View
import kotlin.math.cos
import kotlin.math.sin

/**
 * Vista tipo radar (vista cenital). El telefono esta en el origen (abajo al centro)
 * y mira hacia arriba. Cada punto es (lateral, adelante) en metros.
 */
class RadarView(context: Context) : View(context) {

    var maxRangeM = 5f
    var fovRad = Math.toRadians(70.0).toFloat()

    private val history = ArrayDeque<FloatArray>()
    private val maxFrames = 8

    private val bg = Paint().apply { color = Color.rgb(0, 12, 4) }
    private val grid = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.argb(160, 0, 200, 70); style = Paint.Style.STROKE; strokeWidth = 2f
    }
    private val label = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.argb(200, 0, 255, 90); textSize = 30f
    }
    private val dot = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.FILL }
    private val arc = RectF()

    /** Distancia al eco mas cercano del sonar (m), o null si no hay. */
    @Volatile var sonarRange: Float? = null
        set(value) { field = value; postInvalidate() }
    private val sonarPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.argb(220, 0, 220, 255); style = Paint.Style.STROKE; strokeWidth = 6f
    }
    private val sonarLabel = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(0, 220, 255); textSize = 30f
    }

    /** points = [lateral0, forward0, lateral1, forward1, ...] en metros */
    fun submit(points: FloatArray) {
        post {
            history.addLast(points)
            while (history.size > maxFrames) history.removeFirst()
            invalidate()
        }
    }

    override fun onDraw(canvas: Canvas) {
        val w = width.toFloat()
        val h = height.toFloat()
        canvas.drawRect(0f, 0f, w, h, bg)

        val cx = w / 2f
        val cy = h - 40f
        val pxPerM = (h - 90f) / maxRangeM

        // Anillos de distancia
        for (m in 1..maxRangeM.toInt()) {
            val r = m * pxPerM
            arc.set(cx - r, cy - r, cx + r, cy + r)
            canvas.drawArc(arc, 180f, 180f, false, grid)
            canvas.drawText("${m}m", cx + 6f, cy - r + 28f, label)
        }
        // Limites del campo de vision
        val half = fovRad / 2f
        val rMax = maxRangeM * pxPerM
        for (s in intArrayOf(-1, 1)) {
            canvas.drawLine(
                cx, cy,
                cx + s * sin(half) * rMax, cy - cos(half) * rMax, grid
            )
        }
        canvas.drawLine(cx, cy, cx, cy - rMax, grid)

        // Eco del sonar (solo distancia, sin direccion): arco cian a lo ancho
        val sr = sonarRange
        if (sr != null && sr in 0.1f..maxRangeM) {
            val r = sr * pxPerM
            arc.set(cx - r, cy - r, cx + r, cy + r)
            canvas.drawArc(arc, 180f, 180f, false, sonarPaint)
            canvas.drawText("sonar %.2f m".format(sr), cx - 90f, cy - r - 8f, sonarLabel)
        }

        // Puntos con desvanecimiento (el mas reciente es el mas brillante)
        val n = history.size
        history.forEachIndexed { idx, pts ->
            val alpha = 60 + (195 * (idx + 1) / n)
            var i = 0
            while (i + 1 < pts.size) {
                val lateral = pts[i]
                val fwd = pts[i + 1]
                val dist = kotlin.math.hypot(lateral, fwd)
                dot.color = colorForDistance(dist, alpha)
                canvas.drawCircle(cx + lateral * pxPerM, cy - fwd * pxPerM, 5f, dot)
                i += 2
            }
        }
        canvas.drawCircle(cx, cy, 10f, Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.WHITE })
    }

    private fun colorForDistance(d: Float, alpha: Int): Int {
        val t = (d / maxRangeM).coerceIn(0f, 1f)
        // cerca = rojo, lejos = verde
        val r = (255 * (1f - t)).toInt()
        val g = (255 * t).toInt().coerceAtLeast(80)
        return Color.argb(alpha, r, g, 40)
    }
}
