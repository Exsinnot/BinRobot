import speech_recognition as sr
from pydub import AudioSegment
from io import BytesIO
import pygame

import random


for index, name in enumerate(sr.Microphone.list_microphone_names()):
    print(f"Microphone with index {index}: {name}")

def amplify_audio(audio_stream, gain_dB=10):
    # Convert the raw audio to AudioSegment for processing
    audio_segment = AudioSegment.from_file(BytesIO(audio_stream.get_wav_data()))
    # Amplify the audio
    amplified_audio = audio_segment + gain_dB
    # Convert back to audio data for recognition
    return sr.AudioData(amplified_audio.raw_data, audio_stream.sample_rate, audio_stream.sample_width)
def convert_to_wav(audio_data):
    """Convert raw audio data to a WAV format suitable for pygame."""
    audio_segment = AudioSegment(
        data=audio_data,
        sample_width=2,  # Assuming 16-bit audio
        frame_rate=44100,  # Adjust according to your microphone's settings
        channels=1  # Mono audio
    )
    wav_io = BytesIO()
    audio_segment.export(wav_io, format="wav")
    wav_io.seek(0)
    return wav_io
def detect_speech_real_time_continuous():
    recognizer = sr.Recognizer()
    recognizer.energy_threshold = 500
    recognizer.dynamic_energy_threshold = True

    with sr.Microphone(device_index=1) as source:
        print("กรุณาพูดคำที่ต้องการตรวจจับ...")

        while True:
            try:
                recognizer.adjust_for_ambient_noise(source, duration=1)
                audio_stream = recognizer.listen(source)

                wav_io = convert_to_wav(audio_stream.get_raw_data())

            # เล่นเสียงที่ถูกบันทึกไว้ก่อนการส่งไปยัง Google API
                pygame.mixer.init()
                pygame.mixer.music.load(wav_io, 'wav')
                pygame.mixer.music.play()
                # รอให้เสียงเล่นเสร็จ
                while pygame.mixer.music.get_busy():
                    continue
                amplified_audio = amplify_audio(audio_stream, gain_dB=20)  # Adjust gain as needed

                print("Google")
                text = recognizer.recognize_google(amplified_audio, language="th-TH", show_all=False)
                print("คำที่ตรวจจับได้คือ: {}".format(text))
                if "ถังขยะ" in text:
                    sound_list = ['test2.wav',"test.wav","test3.wav"]
                    # โหลดและปรับระดับความดังของไฟล์เสียง MP3 ด้วย PyDub
                    mp3_audio = AudioSegment.from_mp3(sound_list[random.randint(0,2)])
                    mp3_audio = mp3_audio + 0  # เพิ่มความดัง 10 dB

                    wav_io = BytesIO()
                    mp3_audio.export(wav_io, format="wav")

                    wav_io.seek(0)
                    pygame.mixer.init()
                    pygame.mixer.music.load(wav_io, 'wav')

                    pygame.mixer.music.play()

                    # รอให้เสียงเล่นเสร็จ
                    while pygame.mixer.music.get_busy():
                        continue
            except sr.UnknownValueError:
                print("ไม่สามารถตรวจจับคำพูด")
            except sr.RequestError as e:
                print("เกิดข้อผิดพลาดในการเชื่อมต่อกับ Google API: {}".format(e))
            except KeyboardInterrupt:
                print("โปรแกรมถูกหยุด")
                break

if __name__ == "__main__":
    detect_speech_real_time_continuous()
