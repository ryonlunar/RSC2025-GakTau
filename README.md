# UAV Rescue System
Sistem UAV Rescue menggunakan ROS2 dengan C++ untuk navigasi drone dalam misi penyelamatan korban.

## **Struktur Sistem**
Sistem ini terdiri dari tiga paket utama:
1. **MapManager** - Mengelola status peta dan penambahan obstacle/korban.
2. **PathPlanner** - Menggunakan algoritma D* untuk perencanaan jalur dinamis.
3. **DroneController** - Mengatur pergerakan drone, energi, dan kapasitas penumpang.

## **Cara Clone dan Build Proyek**

### **1. Clone Repository**
Jalankan perintah berikut untuk menyalin repositori ini ke komputer Anda:
```bash
cd ~  # Pindah ke direktori home atau lokasi yang diinginkan
git clone https://github.com/username/RSC2025-GakTau.git uav_rescue_ws
cd uav_rescue_ws
```

### **2. Build Workspace**
Karena folder `build/`, `install/`, dan `log/` tidak disertakan dalam repositori, Anda perlu melakukan build ulang workspace setelah cloning:
```bash
colcon build --symlink-install
```
Jika build berhasil, Anda akan melihat hasilnya tanpa error.

### **3. Source Setup File**
Sebelum menjalankan ROS 2, pastikan untuk men-source environment:
```bash
source install/setup.bash
```
Agar tidak perlu menjalankan perintah ini setiap kali membuka terminal, Anda dapat menambahkannya ke `~/.bashrc`:
```bash
echo "source ~/uav_rescue_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### **4. Menjalankan Sistem**
Jalankan seluruh sistem dengan perintah berikut:
```bash
ros2 launch uav_rescue uav_rescue_launch.py
```

### **5. Menjalankan Node Secara Terpisah (Opsional)**
Jika ingin menjalankan setiap node secara terpisah untuk debugging:
```bash
ros2 run map_manager map_node
ros2 run path_planner path_planner_node
ros2 run drone_controller drone_controller_node
```

### **6. Menambahkan Obstacle dan Korban Secara Dinamis**
Gunakan service call untuk menambahkan obstacle atau korban saat simulasi berjalan:
```bash
ros2 service call /add_obstacle uav_interfaces/srv/AddObstacle "{x: 5, y: 6, time: 10.0}"
ros2 service call /add_victim uav_interfaces/srv/AddVictim "{x: 3, y: 4, time: 5.0, priority: 1}"
```

### **7. Menjalankan Misi Penyelamatan**
Untuk mengirim perintah rescue ke UAV:
```bash
ros2 action send_goal /rescue_mission uav_interfaces/action/RescueMission "{target_x: 3, target_y: 4}"
```

---

Dengan mengikuti langkah-langkah di atas, Anda dapat menjalankan dan menguji sistem UAV Rescue secara penuh. Jika ada kendala, silakan periksa log atau jalankan ulang build dan source setup file.

🚀 Selamat mencoba!