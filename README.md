# UAV Rescue System
Sistem UAV Rescue menggunakan ROS2 dengan C++ untuk navigasi drone dalam misi penyelamatan korban.

## **Struktur Sistem**
Sistem ini terdiri dari tiga paket utama:
1. **MapManager** - Mengelola status peta dan penambahan obstacle/korban.
2. **PathPlanner** - Menggunakan algoritma D* untuk perencanaan jalur dinamis.
3. **DroneController** - Mengatur pergerakan drone, energi, dan kapasitas penumpang.