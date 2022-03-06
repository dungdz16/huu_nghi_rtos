# Thiết bị báo động ở biên giới
## Tính năng:
-	Đo bụi 
-	Đo nhiệt độ môi trường
-	Đo độ ẩm môi trường
-	Nhận biết mưa
-	Đo độ ẩm đất
-	Đo khí CO
-	Set thời gian đẩy mẫu lên broker
-	Gửi vị trí
-	Nút ấn SOS: mềm + cứng
-	2 nút ấn tăng giảm tốc độ xylanh điện

Node|Property|Datatype|Ý nghĩa
----|--------|---------|------|
environment|temperature|float|nhiệt độ môi trường
""|humidity|float|độ ẩm môi trường
""|rain|boolean|phát hiện mưa
""|co_gas|float|nồng độ khí CO
""|solid_humid|float|độ ẩm đất
device|location|GeoWithTime|vị trí tháp
""|up_button|boolean|Nút ấn lên xy-lanh
""|down_button|boolean|Nút ấn xuống xy lanh

## Kiểu dữ liệu Payload hỗ trợ

- Payload phải được gửi dưới dạng chuỗi được mã hóa UTF-8
- Dữ liệu Payload phải thuộc 1 trong các kiểu dữ liệu sau

1. String
1. Integer
1. Float
1. Percent
1. Boolean
1. Enum
1. Color
1. DateTime
   - Theo chuẩn ISO 8601
1. GeoWithTime
   - Dữ liệu dạng JSON gồm các Key sau
     | Key |Data Type|Ý nghĩa|
     |-----------|---------|-------|
     |long |Float |Kinh độ|
     |lat |Float |Vĩ độ|
     |time |DateTime |Thời điểm gửi tọa độ|


