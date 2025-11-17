using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL;
using OpenTK.Input;
using System;
namespace RobotArm3D
{
    public class RobotArm : GameWindow
    {
        float theta1 = 0f;
        float theta2 = 0f;
        float theta3 = 0f;
        float theta4 = 0f;
        float link1 = 1.0f;
        float link2 = 0.8f;
        float link3 = 0.6f;
        float link4 = 0.4f;
        bool gripperOpen = true;
        float gripperAngle = 15f;
        Vector3 ballPosition;
        float ballVelocityY = 0f;
        float gravity = -9.8f;
        float ballRadius = 0.15f;
        bool ballCaught = false;
        Vector3 ballGripperOffset;
        float bounceDamping = 0.6f;
        float camAngleX = 35f;
        float camAngleY = 45f;
        float camDistance = 7f;
        float minReachRadius;
        float maxReachRadius;
        Random random = new Random();
        bool laserActive = false;
        System.Collections.Generic.List<Vector3> laserTrail = new System.Collections.Generic.List<Vector3>();
        bool precisionMode = false;
        bool spaceWasPressed = false;
        public RobotArm()
            : base(1024, 768, GraphicsMode.Default, "3-Joint Robotic Arm")
        {
            VSync = VSyncMode.On;
            CalculateReachLimits();
            SpawnBallAtRandomPosition();
        }
        private void CalculateReachLimits()
        {
            maxReachRadius = link2 + link3 - 0.3f;
            minReachRadius = Math.Abs(link2 - link3) + 0.5f;
        }
        protected override void OnLoad(EventArgs e)
        {
            base.OnLoad(e);
            GL.ClearColor(Color4.Black);
            GL.Enable(EnableCap.DepthTest);
            GL.Enable(EnableCap.Lighting);
            GL.Enable(EnableCap.Light0);
            GL.Enable(EnableCap.ColorMaterial);
            GL.ColorMaterial(MaterialFace.FrontAndBack, ColorMaterialParameter.AmbientAndDiffuse);
            float[] lightAmbient = { 0.3f, 0.3f, 0.3f, 1f };
            float[] lightDiffuse = { 1f, 1f, 1f, 1f };
            float[] lightPos = { 5f, 8f, 10f, 1f };
            GL.Light(LightName.Light0, LightParameter.Ambient, lightAmbient);
            GL.Light(LightName.Light0, LightParameter.Diffuse, lightDiffuse);
            GL.Light(LightName.Light0, LightParameter.Position, lightPos);
            GL.Enable(EnableCap.Normalize);
        }
        protected override void OnResize(EventArgs e)
        {
            base.OnResize(e);
            GL.Viewport(0, 0, Width, Height);
        }
        protected override void OnUpdateFrame(FrameEventArgs e)
        {
            base.OnUpdateFrame(e);
            var key = Keyboard.GetState();
            if (key.IsKeyDown(Key.Escape)) Exit();
            precisionMode = key.IsKeyDown(Key.ShiftLeft) || key.IsKeyDown(Key.ShiftRight);
            float angleSpeed = precisionMode ? 0.1f : 1f;
            if (key.IsKeyDown(Key.Q)) theta1 += angleSpeed;
            if (key.IsKeyDown(Key.A)) theta1 -= angleSpeed;
            if (key.IsKeyDown(Key.W)) theta2 += angleSpeed;
            if (key.IsKeyDown(Key.S)) theta2 -= angleSpeed;
            if (key.IsKeyDown(Key.E)) theta3 += angleSpeed;
            if (key.IsKeyDown(Key.D)) theta3 -= angleSpeed;
            if (key.IsKeyDown(Key.R)) theta4 += angleSpeed;
            if (key.IsKeyDown(Key.F)) theta4 -= angleSpeed;
            theta2 = MathHelper.Clamp(theta2, -90f, 90f);
            theta3 = MathHelper.Clamp(theta3, -90f, 90f);
            bool spaceIsPressed = key.IsKeyDown(Key.Space);
            if (spaceIsPressed && !spaceWasPressed)
            {
                laserActive = !laserActive;
                if (!laserActive)
                {
                    laserTrail.Clear();
                }
            }
            spaceWasPressed = spaceIsPressed;
            if (laserActive)
            {
                UpdateLaserTrail();
            }
            if (key.IsKeyDown(Key.Z)) gripperOpen = true;
            if (key.IsKeyDown(Key.X)) gripperOpen = false;
            float targetAngle = gripperOpen ? 15f : -15f;
            gripperAngle += (targetAngle - gripperAngle) * 0.1f;
            if (key.IsKeyDown(Key.Left)) camAngleY -= 1f;
            if (key.IsKeyDown(Key.Right)) camAngleY += 1f;
            if (key.IsKeyDown(Key.Up)) camAngleX = Math.Max(-89f, camAngleX - 1f);
            if (key.IsKeyDown(Key.Down)) camAngleX = Math.Min(89f, camAngleX + 1f);
            if (key.IsKeyDown(Key.PageUp)) camDistance = Math.Max(2f, camDistance - 0.1f);
            if (key.IsKeyDown(Key.PageDown)) camDistance = Math.Min(12f, camDistance + 0.1f);
            camAngleX = MathHelper.Clamp(camAngleX, -89f, 89f);
            UpdateBallPhysics((float)e.Time);
        }
        private void UpdateBallPhysics(float deltaTime)
        {
            if (ballCaught)
            {
                UpdateCaughtBallPosition();
            }
            else
            {
                // Yerçekimi uygula
                ballVelocityY += gravity * deltaTime;

                // Pozisyonu güncelle - KOLDAN BAĞIMSIZ hareket
                ballPosition.Y += ballVelocityY * deltaTime;

                // Zemin çarpışması - top zemine çarptığında zıplama
                if (ballPosition.Y <= ballRadius)
                {
                    ballPosition.Y = ballRadius;

                    // Zıplama - hızı azaltarak ters çevir
                    if (ballVelocityY < 0)
                    {
                        ballVelocityY = -ballVelocityY * bounceDamping;

                        // Hız çok düştüğünde dur
                        if (Math.Abs(ballVelocityY) < 0.1f)
                        {
                            ballVelocityY = 0f;
                        }
                    }
                }

                // Yakalama kontrolü
                CheckBallCatch();
            }
        }
        private void CheckBallCatch()
        {
            // Pençe durumu kontrolü
            Vector3 endEffectorPos = CalculateEndEffectorPosition();
            float distance = (ballPosition - endEffectorPos).Length;

            // Her frame debug (sadece yakınsa)
            if (!gripperOpen && distance < 1.0f && !ballCaught)
            {
                System.Console.WriteLine($"DEBUG: gripperOpen={gripperOpen}, gripperAngle={gripperAngle:F2}, distance={distance:F3}, ballCaught={ballCaught}");
            }

            // ÇOK BASIT KOŞUL: Sadece pençe kapatma tuşu basılı ve top yakında mı?
            bool tryingToCatch = !gripperOpen;
            bool ballNearby = distance < 0.6f;

            if (tryingToCatch && ballNearby && !ballCaught)
            {
                ballCaught = true;
                ballVelocityY = 0f;

                // Topu pençenin lokal koordinat sisteminde sakla
                Matrix4 invTransform = GetGripperTransform();
                invTransform.Invert();
                Vector4 localPos = Vector4.Transform(new Vector4(ballPosition.X, ballPosition.Y, ballPosition.Z, 1), invTransform);
                ballGripperOffset = new Vector3(localPos.X, localPos.Y, localPos.Z);

                System.Console.WriteLine($"*** TOP YAKALANDI! *** Mesafe: {distance:F3}, GripperAngle: {gripperAngle:F2}");
            }

            // Pençe açıldıysa topu bırak
            if (ballCaught && gripperOpen)
            {
                ballCaught = false;
                System.Console.WriteLine("*** TOP BIRAKILDI! ***");
            }
        }
        private void UpdateCaughtBallPosition()
        {
            // Topu pençe ile birlikte taşı
            Matrix4 transform = GetGripperTransform();
            Vector4 worldPos = Vector4.Transform(new Vector4(ballGripperOffset.X, ballGripperOffset.Y, ballGripperOffset.Z, 1), transform);
            ballPosition = new Vector3(worldPos.X, worldPos.Y, worldPos.Z);
            ballVelocityY = 0f;
        }
        private Matrix4 GetGripperTransform()
        {
            Matrix4 transform = Matrix4.Identity;
            transform *= Matrix4.CreateRotationY(MathHelper.DegreesToRadians(theta1));
            transform *= Matrix4.CreateTranslation(0, link1, 0);
            transform *= Matrix4.CreateRotationZ(MathHelper.DegreesToRadians(theta2));
            transform *= Matrix4.CreateTranslation(0, link2, 0);
            transform *= Matrix4.CreateRotationZ(MathHelper.DegreesToRadians(theta3));
            transform *= Matrix4.CreateTranslation(0, link3, 0);
            transform *= Matrix4.CreateRotationY(MathHelper.DegreesToRadians(theta4));
            return transform;
        }
        private void SpawnBallAtRandomPosition()
        {
            float angle = (float)(random.NextDouble() * Math.PI * 2);
            float radius = minReachRadius + (float)(random.NextDouble() * (maxReachRadius - minReachRadius));
            float x = (float)Math.Cos(angle) * radius;
            float z = (float)Math.Sin(angle) * radius;
            ballPosition = new Vector3(x, 2.5f, z);
            ballVelocityY = 0f;
            ballCaught = false;
        }
        private Vector3 CalculateEndEffectorPosition()
        {
            Matrix4 transform = Matrix4.Identity;
            transform *= Matrix4.CreateRotationY(MathHelper.DegreesToRadians(theta1));
            transform *= Matrix4.CreateTranslation(0, link1, 0);
            transform *= Matrix4.CreateRotationZ(MathHelper.DegreesToRadians(theta2));
            transform *= Matrix4.CreateTranslation(0, link2, 0);
            transform *= Matrix4.CreateRotationZ(MathHelper.DegreesToRadians(theta3));
            transform *= Matrix4.CreateTranslation(0, link3, 0);
            transform *= Matrix4.CreateRotationY(MathHelper.DegreesToRadians(theta4));
            return new Vector3(transform.M41, transform.M42, transform.M43);
        }
        private bool CheckCollisionWithArm(Vector3 ballPos, out Vector3 collisionNormal)
        {
            collisionNormal = Vector3.Zero;
            float collisionRadius = ballRadius + 0.20f; // Top yarıçapı + kol kalınlığı (artırıldı)

            // Kol segmentlerinin pozisyonlarını DOĞRU hesapla
            Vector3 joint0 = Vector3.Zero;

            // Joint 1 - sadece theta1 kadar Y ekseninde döndürüldükten sonra link1 kadar yukarı
            Matrix4 t1 = Matrix4.CreateRotationY(MathHelper.DegreesToRadians(theta1)) * Matrix4.CreateTranslation(0, link1, 0);
            Vector3 joint1 = new Vector3(t1.M41, t1.M42, t1.M43);

            // Joint 2 - önceki transformu al, Z ekseninde dön, sonra link2 kadar ilerle
            Matrix4 t2 = t1 * Matrix4.CreateRotationZ(MathHelper.DegreesToRadians(theta2)) * Matrix4.CreateTranslation(0, link2, 0);
            Vector3 joint2 = new Vector3(t2.M41, t2.M42, t2.M43);

            // Joint 3
            Matrix4 t3 = t2 * Matrix4.CreateRotationZ(MathHelper.DegreesToRadians(theta3)) * Matrix4.CreateTranslation(0, link3, 0);
            Vector3 joint3 = new Vector3(t3.M41, t3.M42, t3.M43);

            // End effector
            Matrix4 t4 = t3 * Matrix4.CreateRotationY(MathHelper.DegreesToRadians(theta4));
            Vector3 endEffector = new Vector3(t4.M41, t4.M42, t4.M43);

            // Her segment için çarpışma kontrolü
            var segments = new System.Collections.Generic.List<(Vector3 start, Vector3 end)>
            {
                (joint0, joint1),
                (joint1, joint2),
                (joint2, joint3),
                (joint3, endEffector)
            };

            foreach (var segment in segments)
            {
                Vector3 segStart = segment.start;
                Vector3 segEnd = segment.end;

                // Segmente en yakın noktayı bul
                Vector3 segDir = segEnd - segStart;
                float segLength = segDir.Length;
                if (segLength < 0.001f) continue;

                segDir = segDir.Normalized();
                Vector3 ballToStart = ballPos - segStart;
                float t = Vector3.Dot(ballToStart, segDir);
                t = MathHelper.Clamp(t, 0, segLength);

                Vector3 closestPoint = segStart + segDir * t;
                float distance = (ballPos - closestPoint).Length;

                if (distance < collisionRadius)
                {
                    collisionNormal = (ballPos - closestPoint).Normalized();
                    return true;
                }
            }

            // Eklemlerle çarpışma
            Vector3[] joints = new[] { joint0, joint1, joint2, joint3, endEffector };
            foreach (var joint in joints)
            {
                float distance = (ballPos - joint).Length;
                if (distance < collisionRadius)
                {
                    collisionNormal = (ballPos - joint).Normalized();
                    return true;
                }
            }

            return false;
        }

        private void UpdateLaserTrail()
        {
            Vector3 endEffectorPos = CalculateEndEffectorPosition();
            Vector3 laserGroundPoint = new Vector3(endEffectorPos.X, 0.01f, endEffectorPos.Z);
            if (laserTrail.Count == 0 || (laserTrail[laserTrail.Count - 1] - laserGroundPoint).Length > 0.01f)
            {
                laserTrail.Add(laserGroundPoint);
                if (laserTrail.Count > 5000)
                {
                    laserTrail.RemoveAt(0);
                }
            }
        }
        protected override void OnRenderFrame(FrameEventArgs e)
        {
            base.OnRenderFrame(e);
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);
            Matrix4 proj = Matrix4.CreatePerspectiveFieldOfView(
                MathHelper.PiOver4, Width / (float)Height, 0.1f, 100f);
            GL.MatrixMode(MatrixMode.Projection);
            GL.LoadMatrix(ref proj);
            Vector3 camPos = CalculateCameraPosition();
            Matrix4 look = Matrix4.LookAt(camPos, Vector3.Zero, Vector3.UnitY);
            GL.MatrixMode(MatrixMode.Modelview);
            GL.LoadMatrix(ref look);
            DrawGround();
            DrawAxes();
            DrawLaserTrail();
            DrawRobotArm();
            DrawBall();
            if (laserActive)
            {
                DrawLaser();
            }
            CalculateAndDisplayEndEffectorPosition();
            SwapBuffers();
        }
        private Vector3 CalculateCameraPosition()
        {
            float xRad = MathHelper.DegreesToRadians(camAngleX);
            float yRad = MathHelper.DegreesToRadians(camAngleY);
            float x = camDistance * (float)Math.Cos(yRad) * (float)Math.Cos(xRad);
            float y = camDistance * (float)Math.Sin(xRad);
            float z = camDistance * (float)Math.Sin(yRad) * (float)Math.Cos(xRad);
            return new Vector3(x, y, z);
        }
        private void CalculateAndDisplayEndEffectorPosition()
        {
            GL.MatrixMode(MatrixMode.Modelview);
            GL.PushMatrix();
            GL.LoadIdentity();
            GL.Rotate(theta1, 0, 1, 0);
            GL.Translate(0, link1, 0);
            GL.Rotate(theta2, 0, 0, 1);
            GL.Translate(0, link2, 0);
            GL.Rotate(theta3, 0, 0, 1);
            GL.Translate(0, link3, 0);
            GL.Rotate(theta4, 0, 1, 0);
            float[] modelviewMatrix = new float[16];
            GL.GetFloat(GetPName.ModelviewMatrix, modelviewMatrix);
            float endX = modelviewMatrix[12];
            float endY = modelviewMatrix[13];
            float endZ = modelviewMatrix[14];
            GL.PopMatrix();
            float totalReach = (float)Math.Sqrt(endX * endX + endY * endY + endZ * endZ);
            string gripperStatus = gripperOpen ? "Acik" : "Kapali";
            string ballStatus = ballCaught ? "Yakalandi" : "Serbest";
            string precisionStatus = precisionMode ? "[HASSAS]" : "";
            string laserStatus = laserActive ? "[LAZER]" : "";

            // Mesafe hesapla
            float distanceToGripper = (ballPosition - new Vector3(endX, endY, endZ)).Length;

            // Yakalama durumu kontrol
            bool canCatch = !gripperOpen && gripperAngle < -8f && distanceToGripper < 0.4f;
            string catchStatus = canCatch ? "[YAKALAYABILIR!]" : "";

            Title = string.Format(
                "3-Joint Robotic Arm | Pence: {0} (Aci:{1:F1}) | Top: {2} | Mesafe: {3:F3} {4}",
                gripperStatus, gripperAngle, ballStatus, distanceToGripper, catchStatus
            );
        }
        void DrawAxes()
        {
            GL.Disable(EnableCap.Lighting);
            GL.LineWidth(4);
            GL.Begin(PrimitiveType.Lines);
            float axisLength = 3.0f;
            GL.Color3(1, 0, 0); GL.Vertex3(0, 0, 0); GL.Vertex3(axisLength, 0, 0);
            GL.Color3(0, 1, 0); GL.Vertex3(0, 0, 0); GL.Vertex3(0, axisLength, 0);
            GL.Color3(0, 0, 1); GL.Vertex3(0, 0, 0); GL.Vertex3(0, 0, axisLength);
            GL.End();
            GL.LineWidth(1);
            GL.Enable(EnableCap.Lighting);
        }
        void DrawGround()
        {
            GL.Disable(EnableCap.Lighting);
            GL.Color3(0.8, 0.8, 0.8);
            GL.Begin(PrimitiveType.Lines);
            float range = 5f;
            for (int i = -5; i <= 5; i++)
            {
                GL.Vertex3(i, 0f, -range); GL.Vertex3(i, 0f, range);
                GL.Vertex3(-range, 0f, i); GL.Vertex3(range, 0f, i);
            }
            GL.End();
            GL.Enable(EnableCap.Lighting);
        }
        void DrawJointHub()
        {
            GL.PushMatrix();
            DrawSphere(0.15f, 16, 16);
            GL.PopMatrix();
        }
        void DrawLink(float length)
        {
            GL.PushMatrix();
            DrawCylinder(0.1f, length, 16);
            GL.PopMatrix();
        }

        void DrawLinkWithStripes(float length)
        {
            GL.PushMatrix();

            // Ana turuncu silindir
            GL.Color3(1.0f, 0.4f, 0.0f);
            DrawCylinder(0.1f, length, 16);

            // Siyah şeritler - iş makinesi tarzı
            GL.Disable(EnableCap.Lighting);
            GL.Color3(0.0f, 0.0f, 0.0f);

            int numStripes = (int)(length / 0.15f);
            for (int i = 0; i < numStripes; i += 2)
            {
                float stripeStart = i * 0.15f;
                float stripeHeight = 0.12f;

                GL.PushMatrix();
                GL.Translate(0, stripeStart, 0);
                DrawCylinder(0.102f, stripeHeight, 16);
                GL.PopMatrix();
            }

            GL.Enable(EnableCap.Lighting);
            GL.PopMatrix();
        }
        void DrawRobotArm()
        {
            GL.PushMatrix();

            // Endüstriyel turuncu renk
            GL.Color3(1.0f, 0.4f, 0.0f);
            DrawJointHub();
            GL.Rotate(theta1, 0, 1, 0);
            DrawLinkWithStripes(link1);
            GL.Translate(0, link1, 0);

            GL.Color3(1.0f, 0.4f, 0.0f);
            DrawJointHub();
            GL.Rotate(theta2, 0, 0, 1);
            DrawLinkWithStripes(link2);
            GL.Translate(0, link2, 0);

            GL.Color3(1.0f, 0.4f, 0.0f);
            DrawJointHub();
            GL.Rotate(theta3, 0, 0, 1);
            DrawLinkWithStripes(link3);
            GL.Translate(0, link3, 0);

            GL.Rotate(theta4, 0, 1, 0);
            DrawGripper();
            GL.PopMatrix();
        }
        void DrawBall()
        {
            GL.PushMatrix();
            GL.Translate(ballPosition.X, ballPosition.Y, ballPosition.Z);
            if (ballCaught)
                GL.Color3(0.2f, 1.0f, 0.2f);
            else
                GL.Color3(1.0f, 0.2f, 0.2f);
            GL.Scale(ballRadius, ballRadius, ballRadius);
            DrawCube();
            GL.PopMatrix();
        }
        void DrawLaser()
        {
            Vector3 endEffectorPos = CalculateEndEffectorPosition();
            Vector3 laserGroundPoint = new Vector3(endEffectorPos.X, 0.01f, endEffectorPos.Z);
            GL.Disable(EnableCap.Lighting);
            GL.Color3(1.0f, 0.0f, 0.0f);
            GL.LineWidth(3);
            GL.Begin(PrimitiveType.Lines);
            GL.Vertex3(endEffectorPos.X, endEffectorPos.Y, endEffectorPos.Z);
            GL.Vertex3(laserGroundPoint.X, laserGroundPoint.Y, laserGroundPoint.Z);
            GL.End();
            GL.LineWidth(1);
            GL.Enable(EnableCap.Lighting);
        }
        void DrawLaserTrail()
        {
            if (laserTrail.Count < 2) return;
            GL.Disable(EnableCap.Lighting);
            GL.Color3(1.0f, 0.5f, 0.0f);
            GL.LineWidth(2);
            GL.Begin(PrimitiveType.LineStrip);
            foreach (var point in laserTrail)
            {
                GL.Vertex3(point.X, point.Y, point.Z);
            }
            GL.End();
            GL.LineWidth(1);
            GL.Enable(EnableCap.Lighting);
        }
        void DrawGripper()
        {
            GL.Color3(0.3, 0.3, 0.3);
            GL.PushMatrix();
            GL.Color3(0.5, 0.5, 0.5);
            GL.Translate(0, 0.05f, 0);
            GL.Scale(0.2, 0.1, 0.08);
            DrawCube();
            GL.PopMatrix();
            GL.PushMatrix();
            GL.Color3(0.3, 0.3, 0.3);
            GL.Translate(0.1f, 0.1f, 0);
            GL.Rotate(-gripperAngle, 0, 0, 1);
            GL.Translate(0, 0.15f, 0);
            GL.Scale(0.04, 0.3, 0.04);
            DrawCube();
            GL.PopMatrix();
            GL.PushMatrix();
            GL.Color3(0.3, 0.3, 0.3);
            GL.Translate(-0.1f, 0.1f, 0);
            GL.Rotate(gripperAngle, 0, 0, 1);
            GL.Translate(0, 0.15f, 0);
            GL.Scale(0.04, 0.3, 0.04);
            DrawCube();
            GL.PopMatrix();
        }
        void DrawSphere(float radius, int slices, int stacks)
        {
            for (int i = 0; i < stacks; i++)
            {
                float lat0 = (float)Math.PI * (-0.5f + (float)i / stacks);
                float z0 = (float)Math.Sin(lat0);
                float zr0 = (float)Math.Cos(lat0);

                float lat1 = (float)Math.PI * (-0.5f + (float)(i + 1) / stacks);
                float z1 = (float)Math.Sin(lat1);
                float zr1 = (float)Math.Cos(lat1);

                GL.Begin(PrimitiveType.QuadStrip);
                for (int j = 0; j <= slices; j++)
                {
                    float lng = 2 * (float)Math.PI * (float)j / slices;
                    float x = (float)Math.Cos(lng);
                    float y = (float)Math.Sin(lng);

                    GL.Normal3(x * zr0 * radius, y * zr0 * radius, z0 * radius);
                    GL.Vertex3(x * zr0 * radius, y * zr0 * radius, z0 * radius);
                    GL.Normal3(x * zr1 * radius, y * zr1 * radius, z1 * radius);
                    GL.Vertex3(x * zr1 * radius, y * zr1 * radius, z1 * radius);
                }
                GL.End();
            }
        }
        void DrawCylinder(float radius, float height, int segments)
        {
            GL.Begin(PrimitiveType.QuadStrip);
            for (int i = 0; i <= segments; i++)
            {
                float angle = (float)(i * 2 * Math.PI / segments);
                float x = radius * (float)Math.Cos(angle);
                float z = radius * (float)Math.Sin(angle);
                float nx = (float)Math.Cos(angle);
                float nz = (float)Math.Sin(angle);
                GL.Normal3(nx, 0, nz);
                GL.Vertex3(x, 0, z);
                GL.Vertex3(x, height, z);
            }
            GL.End();
            GL.Begin(PrimitiveType.TriangleFan);
            GL.Normal3(0, 1, 0);
            GL.Vertex3(0, height, 0);
            for (int i = 0; i <= segments; i++)
            {
                float angle = (float)(i * 2 * Math.PI / segments);
                float x = radius * (float)Math.Cos(angle);
                float z = radius * (float)Math.Sin(angle);
                GL.Vertex3(x, height, z);
            }
            GL.End();
            GL.Begin(PrimitiveType.TriangleFan);
            GL.Normal3(0, -1, 0);
            GL.Vertex3(0, 0, 0);
            for (int i = segments; i >= 0; i--)
            {
                float angle = (float)(i * 2 * Math.PI / segments);
                float x = radius * (float)Math.Cos(angle);
                float z = radius * (float)Math.Sin(angle);
                GL.Vertex3(x, 0, z);
            }
            GL.End();
        }
        void DrawCube()
        {
            GL.Begin(PrimitiveType.Quads);
            GL.Normal3(0, 1, 0);
            GL.Vertex3(-0.5, 0.5, -0.5); GL.Vertex3(0.5, 0.5, -0.5); GL.Vertex3(0.5, 0.5, 0.5); GL.Vertex3(-0.5, 0.5, 0.5);
            GL.Normal3(0, -1, 0);
            GL.Vertex3(-0.5, -0.5, 0.5); GL.Vertex3(0.5, -0.5, 0.5); GL.Vertex3(0.5, -0.5, -0.5); GL.Vertex3(-0.5, -0.5, -0.5);
            GL.Normal3(0, 0, 1);
            GL.Vertex3(-0.5, -0.5, 0.5); GL.Vertex3(-0.5, 0.5, 0.5); GL.Vertex3(0.5, 0.5, 0.5); GL.Vertex3(0.5, -0.5, 0.5);
            GL.Normal3(0, 0, -1);
            GL.Vertex3(0.5, -0.5, -0.5); GL.Vertex3(0.5, 0.5, -0.5); GL.Vertex3(-0.5, 0.5, -0.5); GL.Vertex3(-0.5, -0.5, -0.5);
            GL.Normal3(-1, 0, 0);
            GL.Vertex3(-0.5, -0.5, -0.5); GL.Vertex3(-0.5, 0.5, -0.5); GL.Vertex3(-0.5, 0.5, 0.5); GL.Vertex3(-0.5, -0.5, 0.5);
            GL.Normal3(1, 0, 0);
            GL.Vertex3(0.5, -0.5, 0.5); GL.Vertex3(0.5, 0.5, 0.5); GL.Vertex3(0.5, 0.5, -0.5); GL.Vertex3(0.5, -0.5, -0.5);
            GL.End();
        }
        [STAThread]
        public static void Main()
        {
            using (var app = new RobotArm())
            {
                app.Run(60.0);
            }
        }
    }
}
