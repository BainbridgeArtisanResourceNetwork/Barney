from moviepy.editor import VideoFileClip

def play_video_with_audio(video_path):
    """
    Plays a video file with its associated audio.

    Args:
        video_path (str): The path to the video file (e.g., 'my_video.mp4').
    """
    try:
        # Load the video clip
        video_clip = VideoFileClip(video_path)

        # Play the video clip with audio
        video_clip.preview()

        # Careful to use this one - need to get rid of 'q' loop below
        # or you will not be able to exit app
        # video.clip.preview(fullscreen=True)
        
        # Close the clip after playback
        video_clip.close()

    except Exception as e:
        print(f"An error occurred: {e}")

# Example usage:
if __name__ == "__main__":
    play_video_with_audio('tech_lab_dock.mp4')

    while True:
        user_input = input("Enter 'q' to quit: ")

        if user_input == 'r':
            play_video_with_audio('tech_lab_dock.mp4')
        elif user_input == 'q':
            print("Exiting the program.")
            break  # This breaks out of the while loop, ending the program
