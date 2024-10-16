import os
from pydub import AudioSegment

# ตั้งค่าที่อยู่ของโฟลเดอร์ต้นทางและโฟลเดอร์ปลายทาง
input_folder = "/home/user/BinRobot/sound/Not_MP3"
output_base_folder = "/home/user/BinRobot/sound"

# ตรวจสอบว่าโฟลเดอร์ปลายทางมีอยู่แล้วหรือไม่ ถ้าไม่มีก็สร้างใหม่
if not os.path.exists(output_base_folder):
    os.makedirs(output_base_folder)

# ฟังก์ชันสำหรับตรวจสอบและเพิ่มเลขต่อท้ายชื่อไฟล์หากชื่อซ้ำ
def get_unique_filename(folder, filename):
    base_name, extension = os.path.splitext(filename)
    counter = 1
    new_filename = filename

    while os.path.exists(os.path.join(folder, new_filename)):
        new_filename = f"{base_name}_{counter}{extension}"
        counter += 1

    return new_filename

# วนลูปเพื่ออ่านไฟล์ทั้งหมดในโฟลเดอร์ต้นทาง
for filename in os.listdir(input_folder):
    if filename.endswith(".m4a"):  # กรองไฟล์ที่เป็น .m4a เท่านั้น
        # เอาชื่อไฟล์ที่ไม่มีนามสกุล
        name_without_extension = os.path.splitext(filename)[0]

        # สร้างโฟลเดอร์ใหม่ที่มีชื่อตามชื่อไฟล์ (ไม่มีนามสกุล)
        output_folder = os.path.join(output_base_folder, name_without_extension)
        if not os.path.exists(output_folder):
            os.makedirs(output_folder)

        # โหลดและปรับระดับความดังของไฟล์เสียง
        input_file_path = os.path.join(input_folder, filename)
        audio = AudioSegment.from_file(input_file_path, format="m4a")
        audio = audio + 10  # เพิ่มความดัง 10 dB

        # สร้างชื่อไฟล์ใหม่ที่ไม่ซ้ำกันในโฟลเดอร์
        output_filename = get_unique_filename(output_folder, f"{name_without_extension}.mp3")

        # บันทึกไฟล์เป็น MP3 ในโฟลเดอร์ใหม่
        output_file_path = os.path.join(output_folder, output_filename)
        audio.export(output_file_path, format="mp3")

        print(f"บันทึกไฟล์ MP3 ที่ปรับความดังไว้ในโฟลเดอร์: {output_folder}, ชื่อไฟล์: {output_filename}")
