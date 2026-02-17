# STAGE 1: Build Stage (Derleme Ortamı)
FROM ros:humble as builder

# Gerekli derleme araçlarını kur
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    git \
    libacl1-dev \  
    && rm -rf /var/lib/apt/lists/*

# Çalışma alanını oluştur
WORKDIR /ros2_ws

# Proje dosyalarını kopyala
COPY . src/industrial_robot_security

# Bağımlılıkları yükle ve build et
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# STAGE 2: Runtime Stage (Çalışma Ortamı - Daha hafif)
FROM ros:humble-ros-base

# Sadece gerekli çalışma zamanı araçlarını al
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Build aşamasından derlenmiş dosyaları kopyala
WORKDIR /ros2_ws
COPY --from=builder /ros2_ws/install /ros2_ws/install

# --- YENİ EKLENEN KISIM BAŞLANGICI ---
# Pip'i güncelle ve kütüphaneleri kur
COPY requirements.txt /ros2_ws/
RUN pip3 install --upgrade pip
RUN pip3 install -r /ros2_ws/requirements.txt
# --- YENİ EKLENEN KISIM BİTİŞİ ---

# Environment setup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Default komut
CMD ["/bin/bash"]
