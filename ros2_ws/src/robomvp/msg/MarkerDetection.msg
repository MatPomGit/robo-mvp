# Wiadomość reprezentująca wykryty znacznik wizualny (AprilTag lub QR)
# marker_id: unikalny identyfikator markera – odpowiada kluczom w scene.yaml
int32 marker_id

# Współrzędne piksela środka znacznika w obrazie
float32 image_x
float32 image_y

# Przybliżony rozmiar znacznika w pikselach (średnia przekątna kwadratu markera)
# Używany przez marker_pose_estimator do szacowania odległości: z = (f * size_m) / size_px
float32 size
