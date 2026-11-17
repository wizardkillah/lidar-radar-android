package com.lidar.radar

import android.annotation.SuppressLint
import android.media.AudioAttributes
import android.media.AudioFormat
import android.media.AudioRecord
import android.media.AudioTrack
import android.media.MediaRecorder
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

/**
 * Sonar casi ultrasonico: emite un chirp (18-21 kHz) cada ~60 ms en bucle por el
 * parlante y lo correlaciona (filtro adaptado) con lo que graba el microfono.
 * El pico mas fuerte es el camino directo parlante->microfono; los ecos se miden
 * relativos a el. Entrega la distancia al eco mas cercano (sin direccion).
 */
class SonarEngine(private val onRange: (Float?) -> Unit) {

    private val fs = 48000
    private val f0 = 18000.0
    private val f1 = 21000.0
    private val chirpLen = 192                // 4 ms
    private val period = 2880                 // 60 ms -> alcance max ~10 m
    private val window = 3200                 // muestras por lectura
    private val fftN = 4096
    private val maxOff = 1400                 // ~8 m
    private val minOff = 150                  // ~0.54 m (evita la cola del camino directo)
    private val speedOfSound = 343f

    @Volatile private var running = false
    private var thread: Thread? = null
    private var track: AudioTrack? = null
    private var record: AudioRecord? = null

    private val chirp = DoubleArray(chirpLen) { n ->
        val t = n.toDouble() / fs
        val k = (f1 - f0) / (chirpLen.toDouble() / fs)
        val hann = 0.5 - 0.5 * cos(2 * PI * n / (chirpLen - 1))
        hann * sin(2 * PI * (f0 * t + 0.5 * k * t * t))
    }

    private val chRe = DoubleArray(fftN)
    private val chIm = DoubleArray(fftN)

    init {
        for (i in 0 until chirpLen) chRe[i] = chirp[i]
        fft(chRe, chIm, false)
    }

    @SuppressLint("MissingPermission")
    fun start(): Boolean {
        if (running) return true
        try {
            val buf = ShortArray(period)
            for (i in 0 until chirpLen) buf[i] = (chirp[i] * 0.9 * Short.MAX_VALUE).toInt().toShort()

            val t = AudioTrack.Builder()
                .setAudioAttributes(
                    AudioAttributes.Builder()
                        .setUsage(AudioAttributes.USAGE_MEDIA)
                        .setContentType(AudioAttributes.CONTENT_TYPE_MUSIC).build()
                )
                .setAudioFormat(
                    AudioFormat.Builder().setSampleRate(fs)
                        .setEncoding(AudioFormat.ENCODING_PCM_16BIT)
                        .setChannelMask(AudioFormat.CHANNEL_OUT_MONO).build()
                )
                .setTransferMode(AudioTrack.MODE_STATIC)
                .setBufferSizeInBytes(period * 2)
                .build()
            t.write(buf, 0, period)
            t.setLoopPoints(0, period, -1)

            val minBuf = AudioRecord.getMinBufferSize(
                fs, AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT
            )
            val recBuf = maxOf(minBuf, window * 2 * 4)
            val fmt = AudioFormat.Builder().setSampleRate(fs)
                .setEncoding(AudioFormat.ENCODING_PCM_16BIT)
                .setChannelMask(AudioFormat.CHANNEL_IN_MONO).build()
            // UNPROCESSED evita el filtro/AGC que recorta ultrasonido; si no existe, MIC.
            var r = AudioRecord.Builder().setAudioSource(MediaRecorder.AudioSource.UNPROCESSED)
                .setAudioFormat(fmt).setBufferSizeInBytes(recBuf).build()
            if (r.state != AudioRecord.STATE_INITIALIZED) {
                r.release()
                r = AudioRecord.Builder().setAudioSource(MediaRecorder.AudioSource.MIC)
                    .setAudioFormat(fmt).setBufferSizeInBytes(recBuf).build()
            }
            if (r.state != AudioRecord.STATE_INITIALIZED) return false

            track = t
            record = r
            running = true
            t.play()
            r.startRecording()
            thread = Thread(::loop, "sonar").also { it.start() }
            return true
        } catch (e: Exception) {
            stop()
            return false
        }
    }

    fun stop() {
        running = false
        thread?.join(500)
        thread = null
        try { record?.stop() } catch (_: Exception) {}
        record?.release(); record = null
        try { track?.stop() } catch (_: Exception) {}
        track?.release(); track = null
    }

    private fun loop() {
        val pcm = ShortArray(window)
        val re = DoubleArray(fftN)
        val im = DoubleArray(fftN)
        val env = DoubleArray(period)
        val prof = DoubleArray(maxOff)
        var warm = 0

        while (running) {
            val r = record ?: break
            var got = 0
            while (got < window && running) {
                val n = r.read(pcm, got, window - got)
                if (n <= 0) { running = false; break }
                got += n
            }
            if (!running) break

            java.util.Arrays.fill(re, 0.0); java.util.Arrays.fill(im, 0.0)
            for (i in 0 until window) re[i] = pcm[i].toDouble()
            fft(re, im, false)
            for (i in 0 until fftN) {            // X * conj(C)
                val a = re[i]; val b = im[i]
                re[i] = a * chRe[i] + b * chIm[i]
                im[i] = b * chRe[i] - a * chIm[i]
            }
            fft(re, im, true)

            // energia suavizada (~0.25 ms) de la correlacion
            var acc = 0.0
            val w = 12
            val sq = DoubleArray(period) { re[it] * re[it] }
            for (i in 0 until period) {
                acc += sq[i]
                if (i >= w) acc -= sq[i - w]
                env[i] = acc
            }
            var direct = 0
            for (i in 1 until period) if (env[i] > env[direct]) direct = i
            if (env[direct] < 1e3) { onRange(null); continue } // sin senal (volumen 0 / mic recortando)

            for (off in 0 until maxOff) {
                val v = env[(direct + off) % period] / env[direct]
                prof[off] = if (warm == 0) v else 0.7 * prof[off] + 0.3 * v
            }
            warm = 1

            val sorted = prof.copyOfRange(minOff, maxOff).also { it.sort() }
            val floor = sorted[sorted.size / 2].coerceAtLeast(1e-12)
            var found = -1
            for (off in minOff until maxOff - 1) {
                if (prof[off] > floor * 20 && prof[off] >= prof[off - 1] && prof[off] >= prof[off + 1]) {
                    found = off; break
                }
            }
            onRange(if (found < 0) null else found * speedOfSound / (2f * fs))
        }
    }

    // FFT radix-2 iterativa in situ (inversa normalizada).
    private fun fft(re: DoubleArray, im: DoubleArray, inverse: Boolean) {
        val n = re.size
        var j = 0
        for (i in 1 until n) {
            var bit = n shr 1
            while (j and bit != 0) { j = j xor bit; bit = bit shr 1 }
            j = j xor bit
            if (i < j) {
                var t = re[i]; re[i] = re[j]; re[j] = t
                t = im[i]; im[i] = im[j]; im[j] = t
            }
        }
        var len = 2
        while (len <= n) {
            val ang = 2 * PI / len * (if (inverse) 1 else -1)
            val wr = cos(ang); val wi = sin(ang)
            var i = 0
            while (i < n) {
                var cr = 1.0; var ci = 0.0
                for (k in 0 until len / 2) {
                    val ur = re[i + k]; val ui = im[i + k]
                    val vr = re[i + k + len / 2] * cr - im[i + k + len / 2] * ci
                    val vi = re[i + k + len / 2] * ci + im[i + k + len / 2] * cr
                    re[i + k] = ur + vr; im[i + k] = ui + vi
                    re[i + k + len / 2] = ur - vr; im[i + k + len / 2] = ui - vi
                    val nr = cr * wr - ci * wi
                    ci = cr * wi + ci * wr; cr = nr
                }
                i += len
            }
            len = len shl 1
        }
        if (inverse) for (i in 0 until n) { re[i] /= n; im[i] /= n }
    }
}
