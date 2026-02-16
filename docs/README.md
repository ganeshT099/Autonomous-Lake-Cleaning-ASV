# Research & Technical Documentation

This folder contains technical reports and experimental results related to the ALCSV project.

---

## Debris Detection Model Evaluation

**Title:**  
Comparative Analysis of Deep Learning Models for Real-Time Floating Debris Detection in Autonomous Surface Vehicles

### Evaluated Models
- YOLO11n (Original)
- YOLO11n (Fine-tuned)
- Detectron2 (Faster R-CNN)

### Deployment Target
- NVIDIA Jetson orin Nano 8GB

### Key Results
- 69% mAP@50 (Fine-tuned YOLO11n)
- 68% Recall
- 20–25 FPS real-time performance
- 350MB GPU memory usage

This study guided the final model selection for onboard ASV deployment.
