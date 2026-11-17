package com.lidar.radar

import kotlin.math.floor

/**
 * Nube de puntos acumulada en coordenadas del mundo (ARCore: Y arriba).
 *
 * Filtro de ruido: cada voxel (4 cm) debe ser observado [minHits] veces antes de
 * convertirse en punto; el punto final es el PROMEDIO de las observaciones. Asi se
 * eliminan los puntos sueltos y se afinan las superficies.
 *
 * data = [x, y, z, r, g, b] por punto (r,g,b en 0..1).
 */
class PointCloud(
    val maxPoints: Int = 700_000,
    private val voxel: Float = 0.04f,
    private val minHits: Int = 4
) {
    val data = FloatArray(maxPoints * STRIDE)

    @Volatile var count = 0
        private set
    @Volatile var generation = 0
        private set

    // Tabla hash abierta: clave de voxel -> indice de pendiente (>=0), EMITTED o EMPTY
    private val keys = LongArray(CAP)
    private val vals = IntArray(CAP) { EMPTY }
    private var used = 0

    // pendientes: [sx, sy, sz, n, r, g, b, -]
    private val pend = FloatArray(PEND_MAX * 8)
    private var pendNext = 0

    private var sx = 0.0
    private var sy = 0.0
    private var sz = 0.0

    private fun slot(key: Long): Int {
        var h = ((key * -7046029254386353131L) ushr 43).toInt() and (CAP - 1)
        while (vals[h] != EMPTY && keys[h] != key) h = (h + 1) and (CAP - 1)
        return h
    }

    /** true si se emitio un punto nuevo al mapa. */
    @Synchronized
    fun add(x: Float, y: Float, z: Float, r: Float, g: Float, b: Float, w: Float = 1f): Boolean {
        if (count >= maxPoints) return false
        val ix = floor(x / voxel).toInt().toLong() and MASK
        val iy = floor(y / voxel).toInt().toLong() and MASK
        val iz = floor(z / voxel).toInt().toLong() and MASK
        val key = (ix shl 42) or (iy shl 21) or iz

        var s = slot(key)
        if (vals[s] == EMPTY) {
            if (used >= LIMIT || pendNext >= PEND_MAX) {
                rehashKeepEmitted()
                s = slot(key)
            }
            keys[s] = key
            vals[s] = pendNext
            used++
            val p = pendNext * 8
            // suma ponderada (peso = 1/z^2: lo cercano es mas preciso) y contador de observaciones
            pend[p] = x * w; pend[p + 1] = y * w; pend[p + 2] = z * w; pend[p + 3] = w
            pend[p + 4] = r; pend[p + 5] = g; pend[p + 6] = b; pend[p + 7] = 1f
            pendNext++
            return false
        }
        val v = vals[s]
        if (v == EMITTED) return false

        val p = v * 8
        pend[p] += x * w; pend[p + 1] += y * w; pend[p + 2] += z * w
        pend[p + 3] += w
        val n = pend[p + 7] + 1f
        pend[p + 7] = n
        if (n < minHits) return false

        val sw = pend[p + 3]
        val ax = pend[p] / sw; val ay = pend[p + 1] / sw; val az = pend[p + 2] / sw
        val o = count * STRIDE
        data[o] = ax; data[o + 1] = ay; data[o + 2] = az
        data[o + 3] = pend[p + 4]; data[o + 4] = pend[p + 5]; data[o + 5] = pend[p + 6]
        vals[s] = EMITTED
        sx += ax; sy += ay; sz += az
        count++
        return true
    }

    /** Descarta los pendientes (ruido que nunca se confirmo) conservando lo emitido. */
    private fun rehashKeepEmitted() {
        val emitted = LongArray(count)
        var k = 0
        for (i in 0 until CAP) if (vals[i] == EMITTED && k < emitted.size) emitted[k++] = keys[i]
        java.util.Arrays.fill(vals, EMPTY)
        used = 0
        pendNext = 0
        for (i in 0 until k) {
            val s = slot(emitted[i])
            keys[s] = emitted[i]; vals[s] = EMITTED; used++
        }
    }

    @Synchronized
    fun clear() {
        java.util.Arrays.fill(vals, EMPTY)
        used = 0
        pendNext = 0
        sx = 0.0; sy = 0.0; sz = 0.0
        count = 0
        generation++
    }

    @Synchronized
    fun centroid(out: FloatArray): Boolean {
        if (count == 0) return false
        out[0] = (sx / count).toFloat(); out[1] = (sy / count).toFloat(); out[2] = (sz / count).toFloat()
        return true
    }

    /** Copia consistente de los datos para exportar. */
    @Synchronized
    fun snapshot(): FloatArray = data.copyOf(count * STRIDE)

    companion object {
        const val STRIDE = 6
        private const val MASK = 0x1FFFFFL
        private const val CAP = 1 shl 21          // 2M entradas
        private const val LIMIT = 1_400_000       // factor de carga ~0.67
        private const val PEND_MAX = 500_000
        private const val EMPTY = -2
        private const val EMITTED = -1
    }
}
