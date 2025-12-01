using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL;
using OpenTK.Input;
using System;


namespace RobotArm3D
{
    // GameWindow sınıfından türetilen ana oyun sınıfımız.
    // OpenTK kütüphanesinde pencere yönetimi, giriş (input) ve render döngüsü bu sınıf üzerinden yapılır.
    public class RobotArm : GameWindow
    {
        // --- ROBOT KOLU EKLEM AÇILARI (DERECE CİNSİNDEN) ---
        float theta1 = 0f;  // Gövdenin kendi etrafında dönüşü (Y ekseni)
        float theta2 = 0f;  // Omuz ekleminin yukarı/aşağı hareketi (Z ekseni)
        float theta3 = 0f;  // Dirsek ekleminin yukarı/aşağı hareketi (Z ekseni)
        float theta4 = 0f;  // Bileğin kendi etrafında dönüşü (Y ekseni)

        // --- KOL UZUNLUKLARI ---
        // Her bir segmentin uzunluğu, hiyerarşik çizimde öteleme (translate) miktarı olarak kullanılır.
        float link1 = 1.0f; // Taban yüksekliği
        float link2 = 0.8f; // Üst kol uzunluğu
        float link3 = 0.6f; // Ön kol uzunluğu
        float link4 = 0.4f; // El/Gripper uzunluğu

        // --- GRIPPER (KISKAÇ) DURUMLARI ---
        bool gripperOpen = true; // Kıskacın açık mı kapalı mı olduğu
        float gripperAngle = 15f; // Kıskacın anlık açısı (animasyon için)

        // --- TOP VE FİZİK DEĞİŞKENLERİ ---
        Vector3 ballPosition;       // Topun 3D dünyadaki konumu
        float ballVelocityY = 0f;   // Topun dikey hızı (yerçekimi için)
        float gravity = -9.8f;      // Yerçekimi ivmesi
        float ballRadius = 0.15f;   // Topun yarıçapı (çarpışma kontrolleri için)
        bool ballCaught = false;    // Top robot tarafından yakalandı mı?
        Vector3 ballGripperOffset;  // Top yakalandığında, kıskaca göre yerel konumu (Local Space)
        float bounceDamping = 0.6f; // Top yere çarptığında enerji kaybı (sekme katsayısı)

        // --- KAMERA KONTROLÜ ---
        float camAngleX = 35f;      // Kameranın yukarı/aşağı açısı (Pitch)
        float camAngleY = 45f;      // Kameranın etrafında dönme açısı (Yaw)
        float camDistance = 7f;     // Kameranın merkeze uzaklığı (Zoom)

        // --- OYUN MANTIĞI VE SINIRLAR ---
        float minReachRadius;       // Robotun uzanabileceği minimum mesafe
        float maxReachRadius;       // Robotun uzanabileceği maksimum mesafe
        Vector3 basketPosition;     // Hedef sepetin konumu
        float basketRadius = 0.4f;  // Sepetin genişliği
        float basketHeight = 0.6f;  // Sepetin yüksekliği
        int score = 0;              // Oyuncunun puanı
        Random random = new Random(); // Rastgele sayı üretici (topu spawn etmek için)

        // --- GÖRSEL EFEKTLER VE KONTROLLER ---
        bool laserActive = false;   // Lazer işaretleyici açık mı?
        // Lazer izini tutan liste (Trail effect)
        System.Collections.Generic.List<Vector3> laserTrail = new System.Collections.Generic.List<Vector3>();
        bool precisionMode = false; // Shift tuşuna basınca hassas kontrol modu
        bool spaceWasPressed = false; // Space tuşunun önceki durumu (Toggle işlemi için)

        // --- KURUCU METOD (CONSTRUCTOR) ---
        public RobotArm()
            : base(1024, 768, GraphicsMode.Default, "3-Joint Robotic Arm Game")
        {
            // VSync (Dikey Senkronizasyon) açılır, yırtılmaları önler.
            VSync = VSyncMode.On;
            // Robotun erişebileceği sınırları hesapla.
            CalculateReachLimits();
            // Oyuna başlarken topu rastgele bir yere koy.
            SpawnBallAtRandomPosition();
        }

        // Robotun fiziksel sınırlarını hesaplar (topun erişilemeyecek bir yere düşmemesi için).
        private void CalculateReachLimits()
        {
            // Maksimum uzanma: Linklerin toplam uzunluğu (biraz tolerans payı ile)
            maxReachRadius = link2 + link3 - 0.3f;
            // Minimum uzanma: Kendi dibine ne kadar girebileceği
            minReachRadius = Math.Abs(link2 - link3) + 0.5f;
            
            // Sepeti robotun maksimum erişim mesafesinin biraz dışına koyuyoruz.
            float basketDistance = maxReachRadius + basketRadius;
            basketPosition = new Vector3(basketDistance, basketRadius, 0f);
        }

        // --- YÜKLEME VE BAŞLATMA (INIT) ---
        // Pencere ilk açıldığında bir kez çalışır. OpenGL ayarları burada yapılır.
        protected override void OnLoad(EventArgs e)
        {
            base.OnLoad(e);
            
            // Arka plan rengini siyah yap.
            GL.ClearColor(Color4.Black);
            
            // Derinlik testini (Z-Buffer) aç. Bu, nesnelerin birbirinin önünde/arkasında doğru görünmesini sağlar.
            GL.Enable(EnableCap.DepthTest);
            
            // Işıklandırma sistemini aktif et.
            GL.Enable(EnableCap.Lighting);
            GL.Enable(EnableCap.Light0); // 0 numaralı varsayılan ışığı aç.
            
            // Malzeme renklerinin ışıkla etkileşime girmesini sağla.
            GL.Enable(EnableCap.ColorMaterial);
            GL.ColorMaterial(MaterialFace.FrontAndBack, ColorMaterialParameter.AmbientAndDiffuse);
            
            // Işık ayarları (Ortam ışığı, Dağılan ışık, Işık pozisyonu)
            float[] lightAmbient = { 0.3f, 0.3f, 0.3f, 1f }; // Gölgeler zifiri karanlık olmasın
            float[] lightDiffuse = { 1f, 1f, 1f, 1f };       // Parlak beyaz ışık
            float[] lightPos = { 5f, 8f, 10f, 1f };          // Işığın konumu
            
            GL.Light(LightName.Light0, LightParameter.Ambient, lightAmbient);
            GL.Light(LightName.Light0, LightParameter.Diffuse, lightDiffuse);
            GL.Light(LightName.Light0, LightParameter.Position, lightPos);
            
            // Normalleri otomatik normalize et (Ölçekleme yapıldığında ışıklandırmanın bozulmaması için).
            GL.Enable(EnableCap.Normalize);
        }

        // --- PENCERE BOYUTLANDIRMA ---
        protected override void OnResize(EventArgs e)
        {
            base.OnResize(e);
            // Çizim alanını pencere boyutuna eşitle.
            GL.Viewport(0, 0, Width, Height);
        }

        // --- OYUN DÖNGÜSÜ (UPDATE) ---
        // Her karede fizik, input ve oyun mantığı burada güncellenir.
        protected override void OnUpdateFrame(FrameEventArgs e)
        {
            base.OnUpdateFrame(e);
            
            // Klavye durumunu al.
            var key = Keyboard.GetState();
            
            // ESC tuşu ile çıkış.
            if (key.IsKeyDown(Key.Escape)) Exit();

            // Shift tuşu basılıysa "Hassas Mod" (daha yavaş hareket).
            precisionMode = key.IsKeyDown(Key.ShiftLeft) || key.IsKeyDown(Key.ShiftRight);
            float angleSpeed = precisionMode ? 0.1f : 1f;

            // --- ROBOT KOLU KONTROLLERİ ---
            // Q/A: Gövde dönüşü (Theta1)
            if (key.IsKeyDown(Key.Q)) theta1 += angleSpeed;
            if (key.IsKeyDown(Key.A)) theta1 -= angleSpeed;
            
            // W/S: Omuz hareketi (Theta2)
            if (key.IsKeyDown(Key.W)) theta2 += angleSpeed;
            if (key.IsKeyDown(Key.S)) theta2 -= angleSpeed;
            
            // E/D: Dirsek hareketi (Theta3)
            if (key.IsKeyDown(Key.E)) theta3 += angleSpeed;
            if (key.IsKeyDown(Key.D)) theta3 -= angleSpeed;
            
            // R/F: Bilek dönüşü (Theta4)
            if (key.IsKeyDown(Key.R)) theta4 += angleSpeed;
            if (key.IsKeyDown(Key.F)) theta4 -= angleSpeed;

            // Açıları mantıklı sınırlarda tut (Kendi içine girmesin).
            theta2 = MathHelper.Clamp(theta2, -90f, 90f);
            theta3 = MathHelper.Clamp(theta3, -90f, 90f);

            // --- LAZER AÇMA/KAPAMA (TOGGLE) ---
            bool spaceIsPressed = key.IsKeyDown(Key.Space);
            if (spaceIsPressed && !spaceWasPressed) // Tuşa yeni basıldıysa (basılı tutulmuyorsa)
            {
                laserActive = !laserActive;
                if (!laserActive)
                {
                    laserTrail.Clear(); // Lazer kapanınca izi temizle.
                }
            }
            spaceWasPressed = spaceIsPressed;

            if (laserActive)
            {
                UpdateLaserTrail();
            }

            // --- KISKAÇ KONTROLÜ ---
            if (key.IsKeyDown(Key.Z)) gripperOpen = true;  // Aç
            if (key.IsKeyDown(Key.X)) gripperOpen = false; // Kapa

            // Kıskacın açılıp kapanmasını animasyonlu yap (Doğrusal interpolasyon - Lerp benzeri).
            float targetAngle = gripperOpen ? 15f : -15f;
            gripperAngle += (targetAngle - gripperAngle) * 0.1f;

            // --- KAMERA KONTROLLERİ ---
            if (key.IsKeyDown(Key.Left)) camAngleY -= 1f;
            if (key.IsKeyDown(Key.Right)) camAngleY += 1f;
            if (key.IsKeyDown(Key.Up)) camAngleX = Math.Max(-89f, camAngleX - 1f);
            if (key.IsKeyDown(Key.Down)) camAngleX = Math.Min(89f, camAngleX + 1f);
            // Zoom (PageUp/PageDown)
            if (key.IsKeyDown(Key.PageUp)) camDistance = Math.Max(2f, camDistance - 0.1f);
            if (key.IsKeyDown(Key.PageDown)) camDistance = Math.Min(12f, camDistance + 0.1f);
            
            camAngleX = MathHelper.Clamp(camAngleX, -89f, 89f); // Kameranın tepetaklak olmasını engelle.

            // Topun fiziğini güncelle.
            UpdateBallPhysics((float)e.Time);
        }

        // --- FİZİK MOTORU ---
        private void UpdateBallPhysics(float deltaTime)
        {
            if (ballCaught)
            {
                // Eğer top yakalandıysa, robot eliyle birlikte hareket etmeli.
                UpdateCaughtBallPosition();
            }
            else
            {
                // --- SERBEST DÜŞÜŞ MANTIĞI ---
                // Hızı yerçekimi ile artır.
                ballVelocityY += gravity * deltaTime;
                // Pozisyonu hız ile değiştir.
                ballPosition.Y += ballVelocityY * deltaTime;

                // --- YERLE ÇARPIŞMA ---
                if (ballPosition.Y <= ballRadius) // Topun altı yere değdi mi?
                {
                    ballPosition.Y = ballRadius; // Yerin altına girmesini engelle.
                    ballVelocityY = -ballVelocityY * bounceDamping; // Ters yönde sek (enerji kaybı ile).
                    
                    // Çok küçük zıplamaları sıfırla (titremeyi önlemek için).
                    if (Math.Abs(ballVelocityY) < 0.1f)
                    {
                        ballVelocityY = 0f;
                    }
                }

                CheckBallCatch();       // Robot topu yakaladı mı?
                CheckBasketScore();     // Top sepete girdi mi?
                CheckBallOutOfBounds(); // Top sahadan çıktı mı?
            }
        }

        // --- TOP YAKALAMA MANTIĞI ---
        private void CheckBallCatch()
        {
            // Kıskaç yeterince kapalı mı?
            bool gripperClosed = gripperAngle < 8f;

            if (gripperClosed && !ballCaught)
            {
                // Robotun uç noktasının (End Effector) konumunu hesapla.
                Vector3 endEffectorPos = CalculateEndEffectorPosition();
                
                // Top ile el arasındaki mesafeyi ölç.
                float distance = (ballPosition - endEffectorPos).Length;

                // Mesafe yeterince yakınsa topu yakala.
                if (distance < 0.7f)
                {
                    ballCaught = true;
                    ballVelocityY = 0f;

                    // --- MATRİS TERSİNİ ALMA (INVERSE KINEMATICS MANTIĞI) ---
                    // Top yakalandığı an, topun gripper'a göre "Local" pozisyonunu saklamamız lazım.
                    // Böylece robot kolu döndüğünde top da o yerel konumunu koruyarak döner.
                    Matrix4 invTransform = GetGripperTransform();
                    invTransform.Invert(); // Dünya koordinatından Yerel koordinata geçiş matrisi.
                    
                    // Topun dünya pozisyonunu, gripper'ın yerel pozisyonuna çevir.
                    Vector4 localPos = Vector4.Transform(new Vector4(ballPosition.X, ballPosition.Y, ballPosition.Z, 1), invTransform);
                    ballGripperOffset = new Vector3(localPos.X, localPos.Y, localPos.Z);
                }
            }
        }

        // --- YAKALANAN TOPUN HAREKETİ ---
        private void UpdateCaughtBallPosition()
        {
            // Eğer oyuncu kıskacı açarsa top düşmeli.
            if (gripperAngle > 10f)
            {
                ballCaught = false;
                return;
            }

            // Gripper'ın şu anki Dünya (World) matrisini al.
            Matrix4 transform = GetGripperTransform();
            
            // Topun sakladığımız yerel ofsetini, bu matrisle çarparak yeni dünya konumunu bul.
            Vector4 worldPos = Vector4.Transform(new Vector4(ballGripperOffset.X, ballGripperOffset.Y, ballGripperOffset.Z, 1), transform);
            ballPosition = new Vector3(worldPos.X, worldPos.Y, worldPos.Z);
            ballVelocityY = 0f; // Eldeyken düşmez.
        }

        // --- FORWARD KINEMATICS (İLERİ KİNEMATİK) MATRİSİ ---
        // Eklem açılarından (theta) uç noktanın dönüşüm matrisini hesaplar.
        private Matrix4 GetGripperTransform()
        {
            Matrix4 transform = Matrix4.Identity; // Birim matris ile başla.

            // Matris çarpım sırası önemlidir (OpenTK/OpenGL'de genelde ters sıra gibi düşünülür ama aslında local->world'dür).
            // 1. Taban dönüşü
            transform *= Matrix4.CreateRotationY(MathHelper.DegreesToRadians(theta1));
            // 2. İlk link boyunca git
            transform *= Matrix4.CreateTranslation(0, link1, 0);
            // 3. Omuz dönüşü
            transform *= Matrix4.CreateRotationZ(MathHelper.DegreesToRadians(theta2));
            // 4. İkinci link boyunca git
            transform *= Matrix4.CreateTranslation(0, link2, 0);
            // 5. Dirsek dönüşü
            transform *= Matrix4.CreateRotationZ(MathHelper.DegreesToRadians(theta3));
            // 6. Üçüncü link boyunca git
            transform *= Matrix4.CreateTranslation(0, link3, 0);
            // 7. Bilek dönüşü
            transform *= Matrix4.CreateRotationY(MathHelper.DegreesToRadians(theta4));

            return transform;
        }

        // --- SKOR KONTROLÜ ---
        private void CheckBasketScore()
        {
            if (!ballCaught)
            {
                // Top ile sepet arasındaki yatay (XZ düzlemi) mesafe.
                float distanceToBasket = new Vector2(ballPosition.X - basketPosition.X, ballPosition.Z - basketPosition.Z).Length;
                float basketBottomY = basketPosition.Y - basketRadius;

                // 1. Yatayda sepetin içinde mi?
                // 2. Dikeyde sepetin içine girmiş mi?
                // 3. Hızı çok düşük mü (Sepette durmuş mu)?
                if (distanceToBasket < basketRadius &&
                    ballPosition.Y <= basketBottomY + ballRadius + 0.1f &&
                    Math.Abs(ballVelocityY) < 0.2f)
                {
                    score++; // Puan ver.
                    SpawnBallAtRandomPosition(); // Yeni top gönder.
                }
            }
        }

        // --- SAHA DIŞI KONTROLÜ ---
        private void CheckBallOutOfBounds()
        {
            if (!ballCaught)
            {
                // Top merkezden çok uzaklaşırsa robot erişemez, resetle.
                float distanceFromCenter = new Vector2(ballPosition.X, ballPosition.Z).Length;
                if (distanceFromCenter > maxReachRadius)
                {
                    score--; // Ceza puanı.
                    SpawnBallAtRandomPosition(); // Reset.
                }
            }
        }

        // --- TOP SPAWN (YENİDEN OLUŞTURMA) ---
        private void SpawnBallAtRandomPosition()
        {
            // Rastgele bir açı seç (0 ile 2PI arası).
            float angle = (float)(random.NextDouble() * Math.PI * 2);
            // Rastgele bir yarıçap seç (Min ve Max erişim arasında).
            float radius = minReachRadius + (float)(random.NextDouble() * (maxReachRadius - minReachRadius));
            
            // Polar koordinattan Kartezyen koordinata (X, Z) çevir.
            float x = (float)Math.Cos(angle) * radius;
            float z = (float)Math.Sin(angle) * radius;
            
            // Topu havadan bırak.
            ballPosition = new Vector3(x, 2.5f, z);
            ballVelocityY = 0f;
            ballCaught = false;
        }

        // --- UÇ NOKTA POZİSYONU HESAPLAMA ---
        // GetGripperTransform ile aynı mantıkta çalışır ama sadece pozisyon vektörünü (X,Y,Z) döndürür.
        private Vector3 CalculateEndEffectorPosition()
        {
            Matrix4 transform = Matrix4.Identity;
            // Sıralı dönüşümler ve ötelemeler:
            transform *= Matrix4.CreateRotationY(MathHelper.DegreesToRadians(theta1));
            transform *= Matrix4.CreateTranslation(0, link1, 0);
            transform *= Matrix4.CreateRotationZ(MathHelper.DegreesToRadians(theta2));
            transform *= Matrix4.CreateTranslation(0, link2, 0);
            transform *= Matrix4.CreateRotationZ(MathHelper.DegreesToRadians(theta3));
            transform *= Matrix4.CreateTranslation(0, link3, 0);
            transform *= Matrix4.CreateRotationY(MathHelper.DegreesToRadians(theta4));
            
            // Matrisin 4. satırı (veya sütunu) pozisyon bilgisini tutar.
            return new Vector3(transform.M41, transform.M42, transform.M43);
        }

        // --- LAZER İZİ GÜNCELLEME ---
        private void UpdateLaserTrail()
        {
            Vector3 endEffectorPos = CalculateEndEffectorPosition();
            // Lazerin yere değdiği noktayı izdüşüm olarak alıyoruz (Y=0.01f).
            Vector3 laserGroundPoint = new Vector3(endEffectorPos.X, 0.01f, endEffectorPos.Z);

            // Eğer hareket ettiyse listeye yeni nokta ekle.
            if (laserTrail.Count == 0 || (laserTrail[laserTrail.Count - 1] - laserGroundPoint).Length > 0.01f)
            {
                laserTrail.Add(laserGroundPoint);
                // Liste çok şişmesin, eskileri sil (Max 5000 nokta).
                if (laserTrail.Count > 5000)
                {
                    laserTrail.RemoveAt(0);
                }
            }
        }

        // --- ÇİZİM DÖNGÜSÜ (RENDER) ---
        protected override void OnRenderFrame(FrameEventArgs e)
        {
            base.OnRenderFrame(e);
            
            // Ekranı temizle (Renk ve Derinlik tamponlarını sıfırla).
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

            // --- PERSPEKTİF AYARLARI ---
            // 3D görünüm matrisi oluşturuluyor (FOV: 45 derece).
            Matrix4 proj = Matrix4.CreatePerspectiveFieldOfView(
                MathHelper.PiOver4, Width / (float)Height, 0.1f, 100f);
            GL.MatrixMode(MatrixMode.Projection);
            GL.LoadMatrix(ref proj);

            // --- KAMERA AYARLARI ---
            // Kameranın nerede olduğu ve nereye baktığı (LookAt).
            Vector3 camPos = CalculateCameraPosition();
            Matrix4 look = Matrix4.LookAt(camPos, Vector3.Zero, Vector3.UnitY);
            GL.MatrixMode(MatrixMode.Modelview);
            GL.LoadMatrix(ref look);

            // --- SAHNE ÇİZİMLERİ ---
            DrawGround();     // Zemini çiz
            DrawAxes();       // Koordinat eksenlerini (X,Y,Z okları) çiz
            DrawBasket();     // i çizSepet
            DrawLaserTrail(); // Lazer izini çiz
            DrawRobotArm();   // Robot kolunu çiz (Hiyerarşik)
            DrawBall();       // Topu çiz
            DrawSpawnRadius();// Topun doğabileceği alan sınırlarını çiz

            if (laserActive)
            {
                DrawLaser(); // Aktif lazer ışınını çiz
            }

            // Pencere başlığına bilgileri yazdır.
            CalculateAndDisplayEndEffectorPosition();

            // Arka planda çizilen resmi ekrana bas (Double Buffering).
            SwapBuffers();
        }

        // Küresel koordinatlardan (AngleX, AngleY, Distance) Kartezyen kamera pozisyonu hesaplar.
        private Vector3 CalculateCameraPosition()
        {
            float xRad = MathHelper.DegreesToRadians(camAngleX);
            float yRad = MathHelper.DegreesToRadians(camAngleY);
            
            float x = camDistance * (float)Math.Cos(yRad) * (float)Math.Cos(xRad);
            float y = camDistance * (float)Math.Sin(xRad);
            float z = camDistance * (float)Math.Sin(yRad) * (float)Math.Cos(xRad);
            return new Vector3(x, y, z);
        }

        // OpenGL matrisinden o anki uç nokta koordinatını okuyup başlığa yazar.
        private void CalculateAndDisplayEndEffectorPosition()
        {
            GL.MatrixMode(MatrixMode.Modelview);
            GL.PushMatrix(); // Mevcut matrisi sakla
            GL.LoadIdentity(); // Matrisi sıfırla

            // Robotun tüm dönüşümlerini sanal olarak uygula
            GL.Rotate(theta1, 0, 1, 0);
            GL.Translate(0, link1, 0);
            GL.Rotate(theta2, 0, 0, 1);
            GL.Translate(0, link2, 0);
            GL.Rotate(theta3, 0, 0, 1);
            GL.Translate(0, link3, 0);
            GL.Rotate(theta4, 0, 1, 0);

            // Oluşan matrisi oku
            float[] modelviewMatrix = new float[16];
            GL.GetFloat(GetPName.ModelviewMatrix, modelviewMatrix);
            
            // Matrisin 12, 13, 14. elemanları X, Y, Z translation değerleridir.
            float endX = modelviewMatrix[12];
            float endY = modelviewMatrix[13];
            float endZ = modelviewMatrix[14];
            GL.PopMatrix(); // Matrisi geri yükle

            float totalReach = (float)Math.Sqrt(endX * endX + endY * endY + endZ * endZ);
            string gripperStatus = gripperOpen ? "Acik" : "Kapali";
            string ballStatus = ballCaught ? "Yakalandi" : "Serbest";
            string precisionStatus = precisionMode ? "[HASSAS]" : "";
            string laserStatus = laserActive ? "[LAZER]" : "";

            // Başlığı güncelle
            Title = string.Format(
                "4-Joint Robotic Arm Game | SKOR: {0} | X: {1:F2} Y: {2:F2} Z: {3:F2} | Uzaklik: {4:F2} | Theta4: {5:F1} | Pence: {6} | Top: {7} {8} {9}",
                score, endX, endY, endZ, totalReach, theta4, gripperStatus, ballStatus, precisionStatus, laserStatus
            );
        }

        // --- YARDIMCI ÇİZİM FONKSİYONLARI ---

        // X(Kırmızı), Y(Yeşil), Z(Mavi) eksenlerini çizer.
        void DrawAxes()
        {
            GL.Disable(EnableCap.Lighting); // Eksenler ışıktan etkilenmemeli, net görünmeli.
            GL.LineWidth(4);
            GL.Begin(PrimitiveType.Lines);
            float axisLength = 3.0f;
            // X Ekseni
            GL.Color3(1, 0, 0); GL.Vertex3(0, 0, 0); GL.Vertex3(axisLength, 0, 0);
            // Y Ekseni
            GL.Color3(0, 1, 0); GL.Vertex3(0, 0, 0); GL.Vertex3(0, axisLength, 0);
            // Z Ekseni
            GL.Color3(0, 0, 1); GL.Vertex3(0, 0, 0); GL.Vertex3(0, 0, axisLength);
            GL.End();
            GL.LineWidth(1);
            GL.Enable(EnableCap.Lighting);
        }

        // Izgara (Grid) şeklinde zemin çizer.
        void DrawGround()
        {
            GL.Disable(EnableCap.Lighting);
            GL.Color3(0.8, 0.8, 0.8);
            GL.Begin(PrimitiveType.Lines);
            float range = 5f;
            for (int i = -5; i <= 5; i++)
            {
                // Z ekseni boyunca çizgiler
                GL.Vertex3(i, 0f, -range); GL.Vertex3(i, 0f, range);
                // X ekseni boyunca çizgiler
                GL.Vertex3(-range, 0f, i); GL.Vertex3(range, 0f, i);
            }
            GL.End();
            GL.Enable(EnableCap.Lighting);
        }

        // Eklem yerlerini temsil eden küre.
        void DrawJointHub()
        {
            GL.PushMatrix();
            DrawSphere(0.15f, 16, 16);
            GL.PopMatrix();
        }

        // Kol parçalarını temsil eden silindir.
        void DrawLink(float length)
        {
            GL.PushMatrix();
            DrawCylinder(0.1f, length, 16);
            GL.PopMatrix();
        }

        // --- ROBOT KOLUNUN ASIL ÇİZİMİ (HİYERARŞİK) ---
        void DrawRobotArm()
        {
            GL.PushMatrix(); // Ana matrisi sakla
            
            // 1. TABAN EKLEMİ
            GL.Color3(1.0f, 0.2f, 0.2f); // Kırmızımsı
            DrawJointHub();
            
            // Gövde Dönüşü (Theta1 - Y ekseni)
            GL.Rotate(theta1, 0, 1, 0); 

            // 1. KOL (LINK 1)
            GL.Color3(0.3f, 0.6f, 1.0f); // Mavi
            DrawLink(link1);
            
            // Kolun ucuna git
            GL.Translate(0, link1, 0); 

            // 2. OMUZ EKLEMİ
            GL.Color3(0.2f, 1.0f, 0.2f); // Yeşil
            DrawJointHub();
            
            // Omuz Dönüşü (Theta2 - Z ekseni)
            GL.Rotate(theta2, 0, 0, 1); 

            // 2. KOL (LINK 2)
            GL.Color3(1.0f, 1.0f, 0.2f); // Sarı
            DrawLink(link2);
            
            // Kolun ucuna git
            GL.Translate(0, link2, 0);

            // 3. DİRSEK EKLEMİ
            GL.Color3(1.0f, 0.2f, 1.0f); // Mor
            DrawJointHub();
            
            // Dirsek Dönüşü (Theta3 - Z ekseni)
            GL.Rotate(theta3, 0, 0, 1);

            // 3. KOL (LINK 3 - Ön kol)
            GL.Color3(1.0f, 0.6f, 0.2f); // Turuncu
            DrawLink(link3);
            
            // Kolun ucuna git
            GL.Translate(0, link3, 0);

            // 4. BİLEK DÖNÜŞÜ (Theta4 - Y ekseni)
            GL.Rotate(theta4, 0, 1, 0);

            // KISKACI ÇİZ
            DrawGripper();

            GL.PopMatrix(); // Ana matrise geri dön
        }

        void DrawBall()
        {
            GL.PushMatrix();
            GL.Translate(ballPosition.X, ballPosition.Y, ballPosition.Z);
            
            // Top yakalandıysa rengi yeşil, değilse kırmızı olsun.
            if (ballCaught)
                GL.Color3(0.2f, 1.0f, 0.2f);
            else
                GL.Color3(1.0f, 0.2f, 0.2f);
            
            GL.Scale(ballRadius, ballRadius, ballRadius);
            DrawCube();
            GL.PopMatrix();
        }

        // Sepeti çiz (Basit tellerden oluşan bir kova gibi).
        void DrawBasket()
        {
            GL.PushMatrix();
            // Sepet konumuna git
            GL.Translate(basketPosition.X, basketPosition.Y - basketRadius, basketPosition.Z);
            GL.Disable(EnableCap.Lighting);
            GL.Color3(1.0f, 0.8f, 0.0f); // Altın sarısı
            GL.LineWidth(3);
            int segments = 20;
            
            // Alt çember
            GL.Begin(PrimitiveType.LineLoop);
            for (int i = 0; i < segments; i++)
            {
                float angle = (float)(i * 2 * Math.PI / segments);
                float x = basketRadius * (float)Math.Cos(angle);
                float z = basketRadius * (float)Math.Sin(angle);
                GL.Vertex3(x, 0, z);
            }
            GL.End();

            // Üst çember
            GL.Begin(PrimitiveType.LineLoop);
            for (int i = 0; i < segments; i++)
            {
                float angle = (float)(i * 2 * Math.PI / segments);
                float x = basketRadius * (float)Math.Cos(angle);
                float z = basketRadius * (float)Math.Sin(angle);
                GL.Vertex3(x, basketHeight, z);
            }
            GL.End();

            // Dikey direkler
            GL.Begin(PrimitiveType.Lines);
            for (int i = 0; i < 8; i++)
            {
                float angle = (float)(i * 2 * Math.PI / 8);
                float x = basketRadius * (float)Math.Cos(angle);
                float z = basketRadius * (float)Math.Sin(angle);
                GL.Vertex3(x, 0, z);
                GL.Vertex3(x, basketHeight, z);
            }
            GL.End();
            GL.LineWidth(1);
            GL.Enable(EnableCap.Lighting);
            GL.PopMatrix();
        }

        // Robotun erişim sınırlarını gösteren daireleri çizer.
        void DrawSpawnRadius()
        {
            GL.PushMatrix();
            GL.Disable(EnableCap.Lighting);
            GL.LineWidth(2);
            int segments = 40;
            
            // İç Çember (Min Reach) - Kırmızı
            GL.Color3(1.0f, 0.0f, 0.0f);
            GL.Begin(PrimitiveType.LineLoop);
            for (int i = 0; i < segments; i++)
            {
                float angle = (float)(i * 2 * Math.PI / segments);
                float x = minReachRadius * (float)Math.Cos(angle);
                float z = minReachRadius * (float)Math.Sin(angle);
                GL.Vertex3(x, 0.01f, z);
            }
            GL.End();

            // Dış Çember (Max Reach) - Yeşil
            GL.Color3(0.0f, 1.0f, 0.0f);
            GL.Begin(PrimitiveType.LineLoop);
            for (int i = 0; i < segments; i++)
            {
                float angle = (float)(i * 2 * Math.PI / segments);
                float x = maxReachRadius * (float)Math.Cos(angle);
                float z = maxReachRadius * (float)Math.Sin(angle);
                GL.Vertex3(x, 0.01f, z);
            }
            GL.End();
            GL.LineWidth(1);
            GL.Enable(EnableCap.Lighting);
            GL.PopMatrix();
        }

        // Robotun ucundan yere inen dik bir çizgi (Nişangah).
        void DrawLaser()
        {
            Vector3 endEffectorPos = CalculateEndEffectorPosition();
            Vector3 laserGroundPoint = new Vector3(endEffectorPos.X, 0.01f, endEffectorPos.Z);
            
            GL.Disable(EnableCap.Lighting);
            GL.Color3(1.0f, 0.0f, 0.0f); // Kırmızı lazer
            GL.LineWidth(3);
            GL.Begin(PrimitiveType.Lines);
            GL.Vertex3(endEffectorPos.X, endEffectorPos.Y, endEffectorPos.Z);
            GL.Vertex3(laserGroundPoint.X, laserGroundPoint.Y, laserGroundPoint.Z);
            GL.End();
            GL.LineWidth(1);
            GL.Enable(EnableCap.Lighting);
        }

        // Lazerin izini çizer (LineStrip olarak).
        void DrawLaserTrail()
        {
            if (laserTrail.Count < 2) return;
            GL.Disable(EnableCap.Lighting);
            GL.Color3(1.0f, 0.5f, 0.0f); // Turuncu iz
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

        // Kıskaç modelini çizer.
        void DrawGripper()
        {
            GL.Color3(0.3, 0.3, 0.3);
            
            // Kıskacın tabanı
            GL.PushMatrix();
            GL.Color3(0.5, 0.5, 0.5);
            GL.Translate(0, 0.05f, 0);
            GL.Scale(0.2, 0.1, 0.08);
            DrawCube();
            GL.PopMatrix();

            // Sol Parmak
            GL.PushMatrix();
            GL.Color3(0.3, 0.3, 0.3);
            GL.Translate(0.1f, 0.1f, 0);
            GL.Rotate(-gripperAngle, 0, 0, 1); // Sol parmak negatif açıyla döner
            GL.Translate(0, 0.15f, 0);
            GL.Scale(0.04, 0.3, 0.04);
            DrawCube();
            GL.PopMatrix();

            // Sağ Parmak
            GL.PushMatrix();
            GL.Color3(0.3, 0.3, 0.3);
            GL.Translate(-0.1f, 0.1f, 0);
            GL.Rotate(gripperAngle, 0, 0, 1); // Sağ parmak pozitif açıyla döner
            GL.Translate(0, 0.15f, 0);
            GL.Scale(0.04, 0.3, 0.04);
            DrawCube();
            GL.PopMatrix();
        }

        // Küre çizimi (Matematiksel formül ile vertex hesaplama).
        void DrawSphere(float radius, int slices, int stacks)
        {
            for (int i = 0; i < stacks; i++)
            {
                // Enlem (Latitude) hesapları
                float lat0 = (float)Math.PI * (-0.5f + (float)i / stacks);
                float z0 = (float)Math.Sin(lat0);
                float zr0 = (float)Math.Cos(lat0);

                float lat1 = (float)Math.PI * (-0.5f + (float)(i + 1) / stacks);
                float z1 = (float)Math.Sin(lat1);
                float zr1 = (float)Math.Cos(lat1);

                GL.Begin(PrimitiveType.QuadStrip);
                for (int j = 0; j <= slices; j++)
                {
                    // Boylam (Longitude) hesapları
                    float lng = 2 * (float)Math.PI * (float)j / slices;
                    float x = (float)Math.Cos(lng);
                    float y = (float)Math.Sin(lng);

                    // Normal vektörler (ışıklandırma için) ve Vertexler
                    GL.Normal3(x * zr0 * radius, y * zr0 * radius, z0 * radius);
                    GL.Vertex3(x * zr0 * radius, y * zr0 * radius, z0 * radius);
                    GL.Normal3(x * zr1 * radius, y * zr1 * radius, z1 * radius);
                    GL.Vertex3(x * zr1 * radius, y * zr1 * radius, z1 * radius);
                }
                GL.End();
            }
        }

        // Silindir çizimi.
        void DrawCylinder(float radius, float height, int segments)
        {
            // Gövde (Yan yüzey)
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

            // Üst kapak (Daire)
            GL.Begin(PrimitiveType.TriangleFan);
            GL.Normal3(0, 1, 0);
            GL.Vertex3(0, height, 0); // Merkez
            for (int i = 0; i <= segments; i++)
            {
                float angle = (float)(i * 2 * Math.PI / segments);
                float x = radius * (float)Math.Cos(angle);
                float z = radius * (float)Math.Sin(angle);
                GL.Vertex3(x, height, z);
            }
            GL.End();

            // Alt kapak (Daire)
            GL.Begin(PrimitiveType.TriangleFan);
            GL.Normal3(0, -1, 0);
            GL.Vertex3(0, 0, 0); // Merkez
            for (int i = segments; i >= 0; i--) // Tersi yönde çiz (Normali aşağı baksın)
            {
                float angle = (float)(i * 2 * Math.PI / segments);
                float x = radius * (float)Math.Cos(angle);
                float z = radius * (float)Math.Sin(angle);
                GL.Vertex3(x, 0, z);
            }
            GL.End();
        }

        // Küp çizimi (Her yüz için ayrı ayrı Quad çizilir).
        void DrawCube()
        {
            GL.Begin(PrimitiveType.Quads);
            // Üst Yüz
            GL.Normal3(0, 1, 0);
            GL.Vertex3(-0.5, 0.5, -0.5); GL.Vertex3(0.5, 0.5, -0.5); GL.Vertex3(0.5, 0.5, 0.5); GL.Vertex3(-0.5, 0.5, 0.5);
            // Alt Yüz
            GL.Normal3(0, -1, 0);
            GL.Vertex3(-0.5, -0.5, 0.5); GL.Vertex3(0.5, -0.5, 0.5); GL.Vertex3(0.5, -0.5, -0.5); GL.Vertex3(-0.5, -0.5, -0.5);
            // Ön Yüz
            GL.Normal3(0, 0, 1);
            GL.Vertex3(-0.5, -0.5, 0.5); GL.Vertex3(-0.5, 0.5, 0.5); GL.Vertex3(0.5, 0.5, 0.5); GL.Vertex3(0.5, -0.5, 0.5);
            // Arka Yüz
            GL.Normal3(0, 0, -1);
            GL.Vertex3(0.5, -0.5, -0.5); GL.Vertex3(0.5, 0.5, -0.5); GL.Vertex3(-0.5, 0.5, -0.5); GL.Vertex3(-0.5, -0.5, -0.5);
            // Sol Yüz
            GL.Normal3(-1, 0, 0);
            GL.Vertex3(-0.5, -0.5, -0.5); GL.Vertex3(-0.5, 0.5, -0.5); GL.Vertex3(-0.5, 0.5, 0.5); GL.Vertex3(-0.5, -0.5, 0.5);
            // Sağ Yüz
            GL.Normal3(1, 0, 0);
            GL.Vertex3(0.5, -0.5, 0.5); GL.Vertex3(0.5, 0.5, 0.5); GL.Vertex3(0.5, 0.5, -0.5); GL.Vertex3(0.5, -0.5, -0.5);
            GL.End();
        }

        // --- ANA PROGRAM GİRİŞİ ---
        [STAThread]
        public static void Main()
        {
            using (var app = new RobotArm())
            {
                // Saniyede 60 kare (FPS) hedefiyle çalıştır.
                app.Run(60.0);
            }
        }
    }
}
