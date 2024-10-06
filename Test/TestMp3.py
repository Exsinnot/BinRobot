import pygame
from pydub import AudioSegment
from io import BytesIO

# โหลดและปรับระดับความดังของไฟล์เสียง MP3 ด้วย PyDub
mp3_audio = AudioSegment.from_mp3("test.wav")
mp3_audio = mp3_audio + 10  # เพิ่มความดัง 10 dB

wav_io = BytesIO()
mp3_audio.export(wav_io, format="wav")

wav_io.seek(0)
pygame.mixer.init()
pygame.mixer.music.load(wav_io, 'wav')

pygame.mixer.music.play()

# รอให้เสียงเล่นเสร็จ
while pygame.mixer.music.get_busy():
    continue
