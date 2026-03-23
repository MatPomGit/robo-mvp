# Wiadomość reprezentująca oszacowaną pozycję 3D wykrytego znacznika
# Pozycja wyrażona w układzie kamery (camera frame)
int32 marker_id

# Pozycja znacznika względem kamery (metry)
# x = prawo, y = dół, z = do przodu (konwencja układu kamery)
float32 x
float32 y
float32 z

# Orientacja znacznika jako kwaternion jednostkowy
# W obecnej implementacji zawsze (0,0,0,1) – brak solvePnP
float32 qx
float32 qy
float32 qz
float32 qw
