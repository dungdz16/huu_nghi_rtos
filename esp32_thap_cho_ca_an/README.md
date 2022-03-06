# Tháp cho cá ăn 
## Tính năng:
-	Đo nồng độ O2
-	Đo độ pH
-	Đo nhiệt độ
-	Set độ mở khoang chứa thức ăn (command topic)
-	Set tốc độ bắn thức ăn (command topic)
-	Set tốc độ cánh quạt cấp oxi (command topic)
-	Bật tắt thiết bị cho tháp trên bờ (set ngưỡng trên web) (command topic)
-	Có nâng hạ xy-lanh điện cho hộp bắn thức ăn (command topic)

Node|Property|Datatype|Ý nghĩa
----|--------|--------|------|
environment (trên hồ)|o2_gas|float|Nồng độ O2 trong nước
""|pH|float|Độ pH trong nước
""|temp|float|Nhiệt độ nước
device (trên hồ)|foot_can|float|Động cơ mở khoang thức ăn
""|foot_tray|float|Động cơ quay khay thức ăn
""|fan|float|Động cơ cánh quạt chân vịt

