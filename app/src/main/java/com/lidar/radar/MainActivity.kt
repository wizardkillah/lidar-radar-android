package com.lidar.radar

import android.Manifest
import android.app.Activity
import android.content.ContentValues
import android.content.pm.PackageManager
import android.graphics.Color
import android.media.Image
import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.opengl.Matrix
import android.os.Build
import android.os.Bundle
import android.provider.MediaStore
import android.view.Gravity
import android.view.MotionEvent
import android.view.ScaleGestureDetector
import android.view.View
import android.view.WindowManager
import android.widget.Button
import android.widget.FrameLayout
import android.widget.LinearLayout
import android.widget.TextView
import com.google.ar.core.Anchor
import com.google.ar.core.ArCoreApk
import com.google.ar.core.Plane
import com.google.ar.core.Config
import com.google.ar.core.Frame
import com.google.ar.core.Session
import com.google.ar.core.TrackingState
import com.google.ar.core.exceptions.CameraNotAvailableException
import com.google.ar.core.exceptions.NotYetAvailableException
import com.google.ar.core.exceptions.UnavailableException
import java.io.BufferedWriter
import java.io.File
import java.io.OutputStreamWriter
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10
import kotlin.math.atan
import kotlin.math.cos
import kotlin.math.sin

class MainActivity : Activity(), GLSurfaceView.Renderer {

    // ---------- ARCore ----------
    private var session: Session? = null
    private var installRequested = false
    private var textureSet = false
    private var textureId = 0

    // ---------- UI ----------
    private lateinit var glView: GLSurfaceView
    private lateinit var radar: RadarView
    private lateinit var status: TextView

    // ---------- Mapa 3D ----------
    private val cloud = PointCloud()
    private var uploaded = 0
    private var lastGeneration = 0
    private var groundY = Float.NaN

    // Compuerta de captura
    private var prevTs = 0L
    private val prevFwd = FloatArray(3)
    private val prevUp = FloatArray(3)
    private val lastIntPos = FloatArray(3)
    private var hasLast = false
    private var gateText = ""
    @Volatile private var fixedView = true

    private val poseM = FloatArray(16)
    private val camPos = FloatArray(3)
    private val traj = ArrayList<Float>()
    @Volatile private var resetRequested = false

    // Camara orbital (se controla con el dedo)
    @Volatile private var yaw = 0f
    @Volatile private var pitch = 0.45f
    @Volatile private var dist = 4f
    private val target = FloatArray(3)
    private val centroid = FloatArray(3)

    // ---------- OpenGL ----------
    private var prog = 0
    private var aPos = 0
    private var aCol = 0
    private var uVP = 0
    private var uModel = 0
    private var uEye = 0
    private var uAtten = 0
    private var uSize = 0
    private var uMode = 0
    private var uGround = 0
    @Volatile private var colorMode = 0   // 0 real, 1 altura, 2 distancia
    private val identM = FloatArray(16).also { Matrix.setIdentityM(it, 0) }

    /** Bloque de nube unido a un ancla de ARCore: si ARCore corrige su mapa, el bloque se reacomoda. */
    private class Chunk(val anchor: Anchor, val start: Int) {
        var end = start
        val a0 = FloatArray(16)
        val a0Inv = FloatArray(16)
        val corr = FloatArray(16).also { Matrix.setIdentityM(it, 0) }
        val tmp = FloatArray(16)
    }
    private val chunks = java.util.concurrent.CopyOnWriteArrayList<Chunk>()
    private var floorFound = false
    private var vbo = 0
    private val proj = FloatArray(16)
    private val view = FloatArray(16)
    private val vp = FloatArray(16)
    private val uploadBuf: FloatBuffer =
        ByteBuffer.allocateDirect(CHUNK_POINTS * PointCloud.STRIDE * 4).order(ByteOrder.nativeOrder()).asFloatBuffer()
    private val lineBuf: FloatBuffer =
        ByteBuffer.allocateDirect(MAX_TRAJ * 3 * 4).order(ByteOrder.nativeOrder()).asFloatBuffer()
    private val dotBuf: FloatBuffer =
        ByteBuffer.allocateDirect(3 * 4).order(ByteOrder.nativeOrder()).asFloatBuffer()

    private val colorLut = Array(64) { i ->
        val hsv = floatArrayOf(240f * (1f - i / 63f), 0.85f, 1f)  // azul (bajo) -> rojo (alto)
        val c = Color.HSVToColor(hsv)
        floatArrayOf(Color.red(c) / 255f, Color.green(c) / 255f, Color.blue(c) / 255f)
    }

    private var lastStatusMs = 0L

    // =====================================================================
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        val dp = resources.displayMetrics.density

        val root = FrameLayout(this)
        glView = GLSurfaceView(this).apply {
            setEGLContextClientVersion(2)
            setEGLConfigChooser(8, 8, 8, 8, 16, 0)
            preserveEGLContextOnPause = true
            setRenderer(this@MainActivity)
            renderMode = GLSurfaceView.RENDERMODE_CONTINUOUSLY
        }
        installTouch(glView)
        root.addView(glView, FrameLayout.LayoutParams(-1, -1))

        // Radar 2D en miniatura (abajo a la izquierda)
        radar = RadarView(this)
        root.addView(
            radar,
            FrameLayout.LayoutParams((300 * dp).toInt(), (170 * dp).toInt(), Gravity.BOTTOM or Gravity.START)
        )

        status = TextView(this).apply {
            setTextColor(Color.rgb(0, 255, 90))
            textSize = 14f
            setPadding(24, 16, 24, 16)
            setBackgroundColor(Color.argb(110, 0, 0, 0))
            text = "Iniciando..."
        }
        root.addView(status, FrameLayout.LayoutParams(-2, -2, Gravity.TOP or Gravity.START))

        val bar = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }
        fun btn(label: String, action: () -> Unit) = Button(this).apply {
            text = label; textSize = 12f; setOnClickListener { action() }
            bar.addView(this, LinearLayout.LayoutParams(-2, -2))
        }
        btn("Reiniciar") { resetRequested = true }
        btn("Guardar PLY") { savePly() }
        btn("Radar") {
            radar.visibility = if (radar.visibility == View.VISIBLE) View.GONE else View.VISIBLE
        }
        btn("Color: real/altura/dist.") { colorMode = (colorMode + 1) % 3 }
        btn("Vista fija / mapa") { fixedView = !fixedView }
        btn("Centrar") { yaw = 0f; pitch = 0.45f; dist = 4f }
        root.addView(bar, FrameLayout.LayoutParams(-2, -2, Gravity.TOP or Gravity.END))

        setContentView(root)
    }

    /** 1 dedo = girar, pellizco = zoom. */
    private fun installTouch(v: View) {
        val scale = ScaleGestureDetector(this, object : ScaleGestureDetector.SimpleOnScaleGestureListener() {
            override fun onScale(d: ScaleGestureDetector): Boolean {
                dist = (dist / d.scaleFactor).coerceIn(0.5f, 40f)
                return true
            }
        })
        var lx = 0f
        var ly = 0f
        v.setOnTouchListener { _, e ->
            scale.onTouchEvent(e)
            when (e.actionMasked) {
                MotionEvent.ACTION_DOWN -> { lx = e.x; ly = e.y }
                MotionEvent.ACTION_MOVE -> if (e.pointerCount == 1 && !scale.isInProgress) {
                    yaw -= (e.x - lx) * 0.008f
                    pitch = (pitch + (e.y - ly) * 0.008f).coerceIn(-1.5f, 1.5f)
                    lx = e.x; ly = e.y
                }
                MotionEvent.ACTION_POINTER_UP, MotionEvent.ACTION_POINTER_DOWN -> { lx = e.x; ly = e.y }
            }
            true
        }
    }

    // =====================================================================
    override fun onResume() {
        super.onResume()
        val need = arrayOf(Manifest.permission.CAMERA, Manifest.permission.RECORD_AUDIO)
            .filter { checkSelfPermission(it) != PackageManager.PERMISSION_GRANTED }
        if (need.isNotEmpty()) {
            requestPermissions(need.toTypedArray(), 1)
            return
        }
        startSession()
        startSonar()
    }

    override fun onRequestPermissionsResult(code: Int, perms: Array<out String>, res: IntArray) {
        super.onRequestPermissionsResult(code, perms, res)
        val camOk = checkSelfPermission(Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED
        val micOk = checkSelfPermission(Manifest.permission.RECORD_AUDIO) == PackageManager.PERMISSION_GRANTED
        if (camOk) startSession() else setStatus("sin permiso de camara (solo sonar)")
        if (micOk) startSonar() else { sonarMsg = "Sonar: sin permiso de microfono"; refreshStatus() }
    }

    private fun startSession() {
        try {
            if (session == null) {
                when (ArCoreApk.getInstance().requestInstall(this, !installRequested)) {
                    ArCoreApk.InstallStatus.INSTALL_REQUESTED -> {
                        installRequested = true
                        return
                    }
                    ArCoreApk.InstallStatus.INSTALLED -> {}
                }
                val s = Session(this)
                val config = Config(s)
                if (s.isDepthModeSupported(Config.DepthMode.AUTOMATIC)) {
                    config.depthMode = Config.DepthMode.AUTOMATIC
                } else {
                    setStatus("Este telefono no soporta ARCore Depth.")
                    s.close()
                    return
                }
                config.updateMode = Config.UpdateMode.LATEST_CAMERA_IMAGE
                config.planeFindingMode = Config.PlaneFindingMode.HORIZONTAL   // para ubicar el piso real
                s.configure(config)
                session = s
                textureSet = false
            }
            session?.resume()
            glView.onResume()
            setStatus("Mueve el telefono despacio para empezar a mapear...")
        } catch (e: UnavailableException) {
            setStatus("ARCore no disponible: ${e.javaClass.simpleName}")
        } catch (e: CameraNotAvailableException) {
            setStatus("Camara no disponible; cierra otras apps que la usen.")
            session = null
        } catch (e: Exception) {
            setStatus("Error: ${e.message}")
        }
    }

    override fun onPause() {
        super.onPause()
        glView.onPause()
        session?.pause()
        sonar?.stop()
        sonar = null
        radar.sonarRange = null
    }

    override fun onDestroy() {
        session?.close()
        session = null
        super.onDestroy()
    }

    // =====================================================================
    // Estado / sonar
    private var arMsg = ""
    @Volatile private var sonarMsg = "Sonar: apagado"
    @Volatile private var arNearest: Float? = null
    private var sonar: SonarEngine? = null

    private fun setStatus(msg: String) {
        arMsg = msg
        refreshStatus()
    }

    private fun refreshStatus() {
        runOnUiThread { status.text = "Mapa 3D: $arMsg\n$sonarMsg" }
    }

    private fun startSonar() {
        if (sonar != null) return
        val s = SonarEngine { range ->
            radar.sonarRange = range
            val ar = arNearest
            sonarMsg = when {
                range == null -> "Sonar: sin eco (volumen multimedia al maximo)"
                ar != null && kotlin.math.abs(ar - range) < 0.4f ->
                    "Sonar: %.2f m | CONFIRMADO por ARCore (%.2f m)".format(range, ar)
                else -> "Sonar: %.2f m".format(range)
            }
            refreshStatus()
        }
        if (s.start()) sonar = s else sonarMsg = "Sonar: no se pudo abrir microfono/parlante"
        refreshStatus()
    }

    // =====================================================================
    // GLSurfaceView.Renderer
    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        val tex = IntArray(1)
        GLES20.glGenTextures(1, tex, 0)
        textureId = tex[0]
        GLES20.glBindTexture(GL_TEXTURE_EXTERNAL_OES, textureId)
        textureSet = false

        prog = buildProgram(VERT, FRAG)
        aPos = GLES20.glGetAttribLocation(prog, "aPos")
        aCol = GLES20.glGetAttribLocation(prog, "aCol")
        uVP = GLES20.glGetUniformLocation(prog, "uVP")
        uModel = GLES20.glGetUniformLocation(prog, "uModel")
        uEye = GLES20.glGetUniformLocation(prog, "uEye")
        uAtten = GLES20.glGetUniformLocation(prog, "uAtten")
        uSize = GLES20.glGetUniformLocation(prog, "uSize")
        uMode = GLES20.glGetUniformLocation(prog, "uMode")
        uGround = GLES20.glGetUniformLocation(prog, "uGround")

        val b = IntArray(1)
        GLES20.glGenBuffers(1, b, 0)
        vbo = b[0]
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vbo)
        GLES20.glBufferData(
            GLES20.GL_ARRAY_BUFFER, cloud.maxPoints * PointCloud.STRIDE * 4, null, GLES20.GL_DYNAMIC_DRAW
        )
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)
        uploaded = 0
        lastGeneration = cloud.generation

        GLES20.glClearColor(0.02f, 0.03f, 0.05f, 1f)
        GLES20.glEnable(GLES20.GL_DEPTH_TEST)
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        GLES20.glViewport(0, 0, width, height)
        Matrix.perspectiveM(proj, 0, 60f, width.toFloat() / height, 0.05f, 100f)
        @Suppress("DEPRECATION")
        val rotation = windowManager.defaultDisplay.rotation
        session?.setDisplayGeometry(rotation, width, height)
    }

    override fun onDrawFrame(gl: GL10?) {
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)

        if (resetRequested) {
            resetRequested = false
            cloud.clear()
            traj.clear()
            for (c in chunks) c.anchor.detach()
            chunks.clear()
            groundY = Float.NaN
            floorFound = false
            hasLast = false
        }

        val s = session
        if (s != null) {
            if (!textureSet) {
                s.setCameraTextureName(textureId)
                textureSet = true
            }
            try {
                processFrame(s.update())
            } catch (e: CameraNotAvailableException) {
                setStatus("Camara no disponible")
            } catch (e: Exception) {
                setStatus("Error: ${e.message}")
            }
        }
        uploadNewPoints()
        drawScene()
    }

    // =====================================================================
    // Profundidad -> nube de puntos 3D
    private fun processFrame(frame: Frame) {
        val camera = frame.camera
        if (camera.trackingState != TrackingState.TRACKING) {
            setStatus("Buscando tracking... mueve el telefono despacio")
            return
        }
        camera.pose.toMatrix(poseM, 0)
        camPos[0] = poseM[12]; camPos[1] = poseM[13]; camPos[2] = poseM[14]
        if (groundY.isNaN()) groundY = camPos[1] - 1.4f   // estimacion inicial (~altura de la mano)
        updateFloor()
        addTrajectory()

        // ---- Compuerta de captura: solo integrar con desplazamiento real y sin giro brusco ----
        val fx0 = -poseM[8]; val fy0 = -poseM[9]; val fz0 = -poseM[10]   // hacia donde mira
        val ux0 = poseM[4]; val uy0 = poseM[5]; val uz0 = poseM[6]       // arriba del telefono
        val ts = frame.timestamp
        var angSpeed = 0f
        if (prevTs != 0L && ts > prevTs) {
            val dt = (ts - prevTs) / 1e9f
            val a1 = kotlin.math.acos((fx0 * prevFwd[0] + fy0 * prevFwd[1] + fz0 * prevFwd[2]).coerceIn(-1f, 1f))
            val a2 = kotlin.math.acos((ux0 * prevUp[0] + uy0 * prevUp[1] + uz0 * prevUp[2]).coerceIn(-1f, 1f))
            angSpeed = maxOf(a1, a2) / dt
        }
        prevTs = ts
        prevFwd[0] = fx0; prevFwd[1] = fy0; prevFwd[2] = fz0
        prevUp[0] = ux0; prevUp[1] = uy0; prevUp[2] = uz0

        val moved = if (!hasLast) Float.MAX_VALUE else {
            val dx = camPos[0] - lastIntPos[0]; val dy = camPos[1] - lastIntPos[1]; val dz = camPos[2] - lastIntPos[2]
            kotlin.math.sqrt(dx * dx + dy * dy + dz * dz)
        }
        val tooFast = angSpeed > MAX_ANG_SPEED
        val enoughMove = moved >= MIN_MOVE_M
        val integrate = !tooFast && enoughMove
        if (integrate) {
            lastIntPos[0] = camPos[0]; lastIntPos[1] = camPos[1]; lastIntPos[2] = camPos[2]
            hasLast = true
        }
        gateText = when {
            tooFast -> "PAUSADO: giras muy rapido"
            !enoughMove -> "CAMINA: hace falta desplazarse"
            else -> "MAPEANDO"
        }

        val intr = camera.imageIntrinsics
        val hfov = 2f * atan(intr.imageDimensions[0] / (2f * intr.focalLength[0]))
        radar.fovRad = hfov

        var depth: Image? = null
        var conf: Image? = null
        var cam: Image? = null
        try {
            cam = try { frame.acquireCameraImage() } catch (e: NotYetAvailableException) { null }
            try {
                depth = frame.acquireRawDepthImage16Bits()
                conf = frame.acquireRawDepthConfidenceImage()
            } catch (e: NotYetAvailableException) {
                depth?.close(); conf?.close(); depth = null; conf = null
            }
            if (depth == null) {
                try {
                    depth = frame.acquireDepthImage16Bits()
                } catch (e: NotYetAvailableException) {
                    setStatus("Esperando datos de profundidad...")
                    return
                }
            }

            val w = depth.width
            val h = depth.height
            val dPlane = depth.planes[0]
            val dBuf = dPlane.buffer.order(ByteOrder.nativeOrder())
            val dRow = dPlane.rowStride
            val cPlane = conf?.planes?.get(0)
            val cBuf = cPlane?.buffer
            val cRow = cPlane?.rowStride ?: 0

            val fx = intr.focalLength[0] * w / intr.imageDimensions[0]
            val fy = intr.focalLength[1] * h / intr.imageDimensions[1]
            val cx = intr.principalPoint[0] * w / intr.imageDimensions[0]
            val cy = intr.principalPoint[1] * h / intr.imageDimensions[1]

            val yP = cam?.planes?.get(0)
            val uP = cam?.planes?.get(1)
            val vP = cam?.planes?.get(2)
            val camW = cam?.width ?: 1
            val camH = cam?.height ?: 1

            val doCloud = integrate
            if (doCloud) ensureChunk(camera.pose)
            val rowA = (h * 0.30f).toInt()
            val rowB = (h * 0.65f).toInt()
            val colMin = FloatArray(w) { Float.MAX_VALUE }
            var added = 0

            for (v in 0 until h) {
                for (u in 0 until w) {
                    val mm = dBuf.getShort(v * dRow + u * 2).toInt() and 0xFFFF
                    if (mm < MIN_RANGE_MM || mm > MAX_RANGE_MM) continue
                    // confianza exigida crece con la distancia (lejos = mas ruido)
                    val needConf = CONF_NEAR + (mm - MIN_RANGE_MM).toFloat() / (MAX_RANGE_MM - MIN_RANGE_MM) * (CONF_FAR - CONF_NEAR)
                    if (cBuf != null && (cBuf.get(v * cRow + u).toInt() and 0xFF) < needConf) continue
                    // Descarta bordes (saltos bruscos de profundidad): son puntos "flotantes"
                    if (u + 1 < w) {
                        val nb = dBuf.getShort(v * dRow + (u + 1) * 2).toInt() and 0xFFFF
                        if (nb > 0 && kotlin.math.abs(nb - mm) > mm * EDGE_RATIO) continue
                    }
                    if (v + 1 < h) {
                        val nb = dBuf.getShort((v + 1) * dRow + u * 2).toInt() and 0xFFFF
                        if (nb > 0 && kotlin.math.abs(nb - mm) > mm * EDGE_RATIO) continue
                    }
                    val z = mm / 1000f

                    if (v in rowA until rowB && z < colMin[u]) colMin[u] = z

                    if (doCloud) {
                        // pixel -> camara ARCore (x der, y arriba, z atras) -> mundo
                        val px = (u + 0.5f - cx) * z / fx
                        val py = -(v + 0.5f - cy) * z / fy
                        val pz = -z
                        val wx = poseM[0] * px + poseM[4] * py + poseM[8] * pz + poseM[12]
                        val wy = poseM[1] * px + poseM[5] * py + poseM[9] * pz + poseM[13]
                        val wz = poseM[2] * px + poseM[6] * py + poseM[10] * pz + poseM[14]
                        var cr = 0.7f; var cg = 0.7f; var cb = 0.7f
                        if (yP != null && uP != null && vP != null) {
                            // color real: pixel de profundidad -> pixel YUV de la camara
                            val cu = (((u + 0.5f) / w) * camW).toInt().coerceIn(0, camW - 1)
                            val cv = (((v + 0.5f) / h) * camH).toInt().coerceIn(0, camH - 1)
                            val yy = yP.buffer.get(cv * yP.rowStride + cu).toInt() and 0xFF
                            val ui = (cv shr 1) * uP.rowStride + (cu shr 1) * uP.pixelStride
                            val vi = (cv shr 1) * vP.rowStride + (cu shr 1) * vP.pixelStride
                            val uu = (uP.buffer.get(ui).toInt() and 0xFF) - 128
                            val vv = (vP.buffer.get(vi).toInt() and 0xFF) - 128
                            cr = (yy + 1.402f * vv).coerceIn(0f, 255f) / 255f
                            cg = (yy - 0.344f * uu - 0.714f * vv).coerceIn(0f, 255f) / 255f
                            cb = (yy + 1.772f * uu).coerceIn(0f, 255f) / 255f
                        }
                        if (cloud.add(wx, wy, wz, cr, cg, cb, 1f / (z * z))) added++
                    }
                }
            }

            chunks.lastOrNull()?.end = cloud.count

            // Radar 2D (vista cenital instantanea)
            val tanHalf = kotlin.math.tan(hfov / 2f)
            val pts = ArrayList<Float>(w)
            var nearest = Float.MAX_VALUE
            var x = 0
            while (x < w) {
                val z = colMin[x]
                if (z != Float.MAX_VALUE) {
                    pts.add(((x + 0.5f) / w - 0.5f) * 2f * tanHalf * z)
                    pts.add(z)
                    if (z < nearest) nearest = z
                }
                x += 2
            }
            radar.submit(pts.toFloatArray())
            arNearest = if (nearest == Float.MAX_VALUE) null else nearest

            val now = System.currentTimeMillis()
            if (now - lastStatusMs > 250) {
                lastStatusMs = now
                val full = if (cloud.count >= cloud.maxPoints) " (MAPA LLENO: guarda y reinicia)" else ""
                val floor = if (floorFound) "piso OK" else "buscando piso"
                setStatus("%s | %,d puntos | +%d | %d bloques | %s%s".format(
                    gateText, cloud.count, added, chunks.size, floor, full))
            }
        } finally {
            depth?.close()
            conf?.close()
            cam?.close()
        }
    }

    /** Piso real: el plano horizontal mirando hacia arriba mas bajo que detecta ARCore. */
    private fun updateFloor() {
        val planes = session?.getAllTrackables(Plane::class.java) ?: return
        var best = Float.NaN
        for (p in planes) {
            if (p.trackingState != TrackingState.TRACKING || p.subsumedBy != null) continue
            if (p.type != Plane.Type.HORIZONTAL_UPWARD_FACING) continue
            val y = p.centerPose.ty()
            if (y < camPos[1] - 0.6f && (best.isNaN() || y < best)) best = y
        }
        if (!best.isNaN()) { groundY = best; floorFound = true }
    }

    /** Abre un bloque nuevo (con su ancla) cada cierto numero de puntos o al alejarse del ancla. */
    private fun ensureChunk(pose: com.google.ar.core.Pose) {
        val cur = chunks.lastOrNull()
        val need = cur == null ||
            (cloud.count - cur.start) >= CHUNK_MAX_POINTS ||
            run {
                val dx = camPos[0] - cur.a0[12]; val dy = camPos[1] - cur.a0[13]; val dz = camPos[2] - cur.a0[14]
                dx * dx + dy * dy + dz * dz > CHUNK_RADIUS * CHUNK_RADIUS
            }
        if (!need) return
        val s = session ?: return
        try {
            val c = Chunk(s.createAnchor(pose), cloud.count)
            pose.toMatrix(c.a0, 0)
            Matrix.invertM(c.a0Inv, 0, c.a0, 0)
            chunks.add(c)
        } catch (e: Exception) {
            // sin ancla disponible: el bloque anterior (o la nube sin corregir) sigue valiendo
        }
    }

    private fun addTrajectory() {
        val n = traj.size
        if (n >= 3) {
            val dx = camPos[0] - traj[n - 3]
            val dy = camPos[1] - traj[n - 2]
            val dz = camPos[2] - traj[n - 1]
            if (dx * dx + dy * dy + dz * dz < 0.0004f) return
        }
        if (traj.size / 3 >= MAX_TRAJ) { repeat(3) { traj.removeAt(0) } }
        traj.add(camPos[0]); traj.add(camPos[1]); traj.add(camPos[2])
    }

    // =====================================================================
    // Render
    private fun uploadNewPoints() {
        if (cloud.generation != lastGeneration) {
            lastGeneration = cloud.generation
            uploaded = 0
        }
        val total = cloud.count
        if (uploaded >= total) return
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vbo)
        synchronized(cloud) {
            while (uploaded < total) {
                val n = minOf(total - uploaded, CHUNK_POINTS)
                uploadBuf.clear()
                uploadBuf.put(cloud.data, uploaded * PointCloud.STRIDE, n * PointCloud.STRIDE)
                uploadBuf.position(0)
                GLES20.glBufferSubData(
                    GLES20.GL_ARRAY_BUFFER,
                    uploaded * PointCloud.STRIDE * 4,
                    n * PointCloud.STRIDE * 4,
                    uploadBuf
                )
                uploaded += n
            }
        }
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)
    }

    private fun drawScene() {
        // Objetivo de la camara orbital: centro del mapa (o la posicion del telefono si esta vacio)
        // Vista fija: la camara orbita siempre el origen (punto de partida). Vista "mapa": el centro de la nube.
        if (!fixedView && cloud.centroid(centroid)) {
            for (i in 0..2) target[i] += (centroid[i] - target[i]) * 0.08f
        } else {
            for (i in 0..2) target[i] += (0f - target[i]) * 0.08f
        }
        val y = yaw; val p = pitch; val d = dist
        val ex = target[0] + d * cos(p) * sin(y)
        val ey = target[1] + d * sin(p)
        val ez = target[2] + d * cos(p) * cos(y)
        Matrix.setLookAtM(view, 0, ex, ey, ez, target[0], target[1], target[2], 0f, 1f, 0f)
        Matrix.multiplyMM(vp, 0, proj, 0, view, 0)

        GLES20.glUseProgram(prog)
        GLES20.glUniformMatrix4fv(uVP, 1, false, vp, 0)
        GLES20.glUniform3f(uEye, ex, ey, ez)
        GLES20.glUniformMatrix4fv(uModel, 1, false, identM, 0)
        GLES20.glUniform1f(uAtten, 0f)

        // Nube de puntos: cada bloque se dibuja con la correccion actual de su ancla
        GLES20.glUniform1f(uGround, if (groundY.isNaN()) 0f else groundY)
        if (uploaded > 0) {
            GLES20.glUniform1f(uMode, colorMode.toFloat())
            GLES20.glUniform1f(uSize, 5f)
            GLES20.glUniform1f(uAtten, 1f)
            GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vbo)
            GLES20.glEnableVertexAttribArray(aPos)
            GLES20.glEnableVertexAttribArray(aCol)
            GLES20.glVertexAttribPointer(aPos, 3, GLES20.GL_FLOAT, false, PointCloud.STRIDE * 4, 0)
            GLES20.glVertexAttribPointer(aCol, 3, GLES20.GL_FLOAT, false, PointCloud.STRIDE * 4, 12)

            var pos = 0
            for (c in chunks) {
                if (c.anchor.trackingState == TrackingState.TRACKING) {
                    c.anchor.pose.toMatrix(c.tmp, 0)
                    Matrix.multiplyMM(c.corr, 0, c.tmp, 0, c.a0Inv, 0)   // correccion = ancla_actual * ancla_original^-1
                }
                if (c.start > pos) {                                      // puntos sin bloque: sin correccion
                    GLES20.glUniformMatrix4fv(uModel, 1, false, identM, 0)
                    GLES20.glDrawArrays(GLES20.GL_POINTS, pos, minOf(c.start, uploaded) - pos)
                }
                val e = minOf(c.end, uploaded)
                if (e > c.start) {
                    GLES20.glUniformMatrix4fv(uModel, 1, false, c.corr, 0)
                    GLES20.glDrawArrays(GLES20.GL_POINTS, c.start, e - c.start)
                }
                pos = maxOf(pos, c.end)
            }
            if (pos < uploaded) {
                GLES20.glUniformMatrix4fv(uModel, 1, false, identM, 0)
                GLES20.glDrawArrays(GLES20.GL_POINTS, pos, uploaded - pos)
            }
            GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)
            GLES20.glUniformMatrix4fv(uModel, 1, false, identM, 0)
            GLES20.glUniform1f(uAtten, 0f)
        }

        // Trayectoria + posicion del telefono (arrays en cliente, color constante)
        GLES20.glUniform1f(uMode, 0f)
        GLES20.glDisableVertexAttribArray(aCol)
        GLES20.glEnableVertexAttribArray(aPos)

        // Referencia fija: grilla en el piso (cuadros de 1 m) y ejes en el origen (X rojo, Y verde, Z azul)
        val gy = if (groundY.isNaN()) -1.4f else groundY
        val grid = FloatArray(9 * 2 * 2 * 3)
        var gi = 0
        for (k in -4..4) {
            grid[gi++] = k.toFloat(); grid[gi++] = gy; grid[gi++] = -4f
            grid[gi++] = k.toFloat(); grid[gi++] = gy; grid[gi++] = 4f
            grid[gi++] = -4f; grid[gi++] = gy; grid[gi++] = k.toFloat()
            grid[gi++] = 4f; grid[gi++] = gy; grid[gi++] = k.toFloat()
        }
        drawLines(grid, 0.22f, 0.25f, 0.3f, GLES20.GL_LINES)
        drawLines(floatArrayOf(0f, 0f, 0f, 0.5f, 0f, 0f), 1f, 0.2f, 0.2f, GLES20.GL_LINES)
        drawLines(floatArrayOf(0f, 0f, 0f, 0f, 0.5f, 0f), 0.2f, 1f, 0.2f, GLES20.GL_LINES)
        drawLines(floatArrayOf(0f, 0f, 0f, 0f, 0f, 0.5f), 0.3f, 0.5f, 1f, GLES20.GL_LINES)
        GLES20.glUniform1f(uSize, 16f)
        drawLines(floatArrayOf(0f, 0f, 0f), 1f, 0f, 1f, GLES20.GL_POINTS)
        val n = traj.size / 3
        if (n >= 2) {
            lineBuf.clear()
            for (v in traj) lineBuf.put(v)
            lineBuf.position(0)
            GLES20.glVertexAttrib3f(aCol, 0.2f, 1f, 0.3f)
            GLES20.glVertexAttribPointer(aPos, 3, GLES20.GL_FLOAT, false, 12, lineBuf)
            GLES20.glDrawArrays(GLES20.GL_LINE_STRIP, 0, n)
        }
        if (n >= 1) {
            // marcador blanco en el telefono
            dotBuf.clear(); dotBuf.put(camPos); dotBuf.position(0)
            GLES20.glUniform1f(uSize, 22f)
            GLES20.glVertexAttrib3f(aCol, 1f, 1f, 1f)
            GLES20.glVertexAttribPointer(aPos, 3, GLES20.GL_FLOAT, false, 12, dotBuf)
            GLES20.glDrawArrays(GLES20.GL_POINTS, 0, 1)
            // punto amarillo 0.5 m hacia donde apunta el telefono
            val fwdX = -poseM[8] * 0.5f + camPos[0]
            val fwdY = -poseM[9] * 0.5f + camPos[1]
            val fwdZ = -poseM[10] * 0.5f + camPos[2]
            dotBuf.clear(); dotBuf.put(fwdX); dotBuf.put(fwdY); dotBuf.put(fwdZ); dotBuf.position(0)
            GLES20.glUniform1f(uSize, 14f)
            GLES20.glVertexAttrib3f(aCol, 1f, 0.9f, 0f)
            GLES20.glDrawArrays(GLES20.GL_POINTS, 0, 1)
        }
    }

    private fun drawLines(pts: FloatArray, r: Float, g: Float, b: Float, mode: Int) {
        lineBuf.clear()
        lineBuf.put(pts)
        lineBuf.position(0)
        GLES20.glVertexAttrib3f(aCol, r, g, b)
        GLES20.glVertexAttribPointer(aPos, 3, GLES20.GL_FLOAT, false, 12, lineBuf)
        GLES20.glDrawArrays(mode, 0, pts.size / 3)
    }

    // =====================================================================
    // Exportar PLY a Descargas
    private fun savePly() {
        val snap = cloud.snapshot()
        val count = snap.size / PointCloud.STRIDE
        if (count == 0) { setStatus("No hay puntos que guardar todavia"); return }
        // aplicar la correccion de deriva de cada bloque
        for (c in chunks) {
            val m = c.corr
            for (k in c.start until minOf(c.end, count)) {
                val o = k * PointCloud.STRIDE
                val x = snap[o]; val y = snap[o + 1]; val z = snap[o + 2]
                snap[o] = m[0] * x + m[4] * y + m[8] * z + m[12]
                snap[o + 1] = m[1] * x + m[5] * y + m[9] * z + m[13]
                snap[o + 2] = m[2] * x + m[6] * y + m[10] * z + m[14]
            }
        }
        setStatus("Guardando $count puntos...")
        Thread {
            try {
                val name = "mapa_${System.currentTimeMillis() / 1000}.ply"
                // Antes de Android 10 no hay MediaStore.Downloads: se guarda en la carpeta de la app.
                val destino = if (Build.VERSION.SDK_INT >= 29) "Descargas/$name"
                              else File(getExternalFilesDir(null), name).absolutePath
                val out = if (Build.VERSION.SDK_INT >= 29) {
                    val values = ContentValues().apply {
                        put(MediaStore.Downloads.DISPLAY_NAME, name)
                        put(MediaStore.Downloads.MIME_TYPE, "application/octet-stream")
                        put(MediaStore.Downloads.RELATIVE_PATH, "Download")
                    }
                    val uri = contentResolver.insert(MediaStore.Downloads.EXTERNAL_CONTENT_URI, values)!!
                    contentResolver.openOutputStream(uri)!!
                } else {
                    File(getExternalFilesDir(null), name).outputStream()
                }
                BufferedWriter(OutputStreamWriter(out)).use { w ->
                    w.write("ply\nformat ascii 1.0\nelement vertex $count\n")
                    w.write("property float x\nproperty float y\nproperty float z\n")
                    w.write("property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
                    var i = 0
                    while (i < snap.size) {
                        w.write(
                            "%.3f %.3f %.3f %d %d %d\n".format(
                                java.util.Locale.US,
                                snap[i], snap[i + 1], snap[i + 2],
                                (snap[i + 3] * 255).toInt(), (snap[i + 4] * 255).toInt(), (snap[i + 5] * 255).toInt()
                            )
                        )
                        i += PointCloud.STRIDE
                    }
                }
                setStatus("Guardado en $destino ($count puntos)")
            } catch (e: Exception) {
                setStatus("Error al guardar: ${e.message}")
            }
        }.start()
    }

    // =====================================================================
    private fun buildProgram(vs: String, fs: String): Int {
        fun sh(type: Int, src: String): Int {
            val s = GLES20.glCreateShader(type)
            GLES20.glShaderSource(s, src)
            GLES20.glCompileShader(s)
            return s
        }
        val p = GLES20.glCreateProgram()
        GLES20.glAttachShader(p, sh(GLES20.GL_VERTEX_SHADER, vs))
        GLES20.glAttachShader(p, sh(GLES20.GL_FRAGMENT_SHADER, fs))
        GLES20.glLinkProgram(p)
        return p
    }

    private companion object {
        const val GL_TEXTURE_EXTERNAL_OES = 0x8D65
        const val MIN_RANGE_MM = 400
        const val MAX_RANGE_MM = 3000      // el ruido crece con el cuadrado de la distancia
        const val CONF_NEAR = 100f         // confianza minima (0..255) a 0,4 m
        const val CONF_FAR = 175f          // confianza minima a 3 m
        const val CHUNK_MAX_POINTS = 8000  // puntos por bloque anclado
        const val CHUNK_RADIUS = 1.5f      // m: nuevo bloque al alejarse del ancla
        const val MAX_ANG_SPEED = 0.45f    // rad/s (~26 grados/s): mas rapido = profundidad mala
        const val MIN_MOVE_M = 0.03f       // desplazamiento minimo entre capturas (paralaje)
        const val EDGE_RATIO = 0.06f       // salto de profundidad maximo entre pixeles vecinos
        const val CHUNK_POINTS = 20_000
        const val MAX_TRAJ = 4000

        const val VERT = """
            uniform mat4 uVP;
            uniform mat4 uModel;
            uniform vec3 uEye;
            uniform float uAtten;
            uniform float uSize;
            uniform float uMode;
            uniform float uGround;
            attribute vec3 aPos;
            attribute vec3 aCol;
            varying vec3 vCol;
            void main() {
                vec4 wp = uModel * vec4(aPos, 1.0);
                float dEye = max(length(wp.xyz - uEye), 0.1);
                vCol = aCol;
                if (uMode > 0.5) {
                    float t = (uMode < 1.5) ? clamp((wp.y - uGround) / 3.0, 0.0, 1.0)
                                            : clamp(1.0 - dEye / 6.0, 0.0, 1.0);
                    vCol = vec3(
                        clamp(min(4.0 * t - 1.5, -4.0 * t + 4.5), 0.0, 1.0),
                        clamp(min(4.0 * t - 0.5, -4.0 * t + 3.5), 0.0, 1.0),
                        clamp(min(4.0 * t + 0.5, -4.0 * t + 2.5), 0.0, 1.0));
                }
                gl_Position = uVP * wp;
                if (uAtten > 0.5) {
                    // puntos cercanos al ojo mas grandes, lejanos mas chicos y oscuros (sensacion de profundidad)
                    gl_PointSize = clamp(uSize * 2.2 / dEye, 2.0, 12.0);
                    vCol *= mix(1.0, 0.5, clamp((dEye - 2.0) / 10.0, 0.0, 1.0));
                } else {
                    gl_PointSize = uSize;
                }
            }
        """
        const val FRAG = """
            precision mediump float;
            varying vec3 vCol;
            void main() {
                vec2 c = gl_PointCoord - vec2(0.5);
                if (dot(c, c) > 0.25) discard;
                gl_FragColor = vec4(vCol, 1.0);
            }
        """
    }
}
