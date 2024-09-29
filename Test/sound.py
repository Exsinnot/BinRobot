import speech_recognition as sr
for index, name in enumerate(sr.Microphone.list_microphone_names()):
    print(f"Microphone with index {index}: {name}")
def detect_speech_real_time_continuous():
    recognizer = sr.Recognizer()

    # ตั้งค่า energy_threshold ให้สูงขึ้นเพื่อให้รับเสียงเบาได้ดีขึ้น
    recognizer.energy_threshold = 300  # คุณสามารถลองปรับค่าให้สูงขึ้นตามสภาพแวดล้อม
    recognizer.dynamic_energy_threshold = False


    with sr.Microphone(device_index=1) as source:
        print("กรุณาพูดคำที่ต้องการตรวจจับ...")

        while True:
            try:
                # ปรับเสียงรอบข้างก่อนรับเสียง
                recognizer.adjust_for_ambient_noise(source, duration=1)

                # รับเสียงจากไมโครโฟน
                audio_stream = recognizer.listen(source)


                # เพิ่มฟังก์ชันตรวจจับเสียงโดยใช้ Google API
                print("Google")
                text = recognizer.recognize_google(audio_stream, language="th-TH", show_all=False)
                print("คำที่ตรวจจับได้คือ: {}".format(text))
            except sr.UnknownValueError:
                print("ไม่สามารถตรวจจับคำพูด")
            except sr.RequestError as e:
                print("เกิดข้อผิดพลาดในการเชื่อมต่อกับ Google API: {}".format(e))
            except KeyboardInterrupt:
                print("โปรแกรมถูกหยุด")
                break

if __name__ == "__main__":
    detect_speech_real_time_continuous()
