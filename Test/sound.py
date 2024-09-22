import speech_recognition as sr

# Initialize recognizer
recognizer = sr.Recognizer()

# Use microphone as the source for input
with sr.Microphone() as source:
    print("Say something!")
    audio = recognizer.listen(source)

# Recognize speech using Google's recognition API
try:
    print("You said: " + recognizer.recognize_google(audio))
except sr.UnknownValueError:
    print("Google Speech Recognition could not understand audio")
except sr.RequestError as e:
    print("Could not request results; {0}".format(e))
