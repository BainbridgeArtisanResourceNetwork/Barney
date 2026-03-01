import speech_recognition as sr
for i, mic in enumerate(sr.Microphone.list_microphone_names()):
    print(f"{i}: {mic}")
# Then use the correct index in your code
# source = sr.Microphone(device_index=1)
