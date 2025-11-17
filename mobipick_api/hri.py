#!/usr/bin/env python3

from typing import List
import rospy
from std_msgs.msg import String


class HRI:
    """Human robot interaction helper using Whisper based ASR and a TTS topic."""

    def __init__(
        self,
        namespace: str,
        recognized_speech_topic: str = "/recognized_speech",
        tts_topic: str = "/speak",
    ):
        # Keep namespace for future use or debugging
        self._namespace = namespace

        self._recognized_speech_topic = recognized_speech_topic
        self._tts_topic = tts_topic

        # Buffer of recognized utterances since last clear
        self._recognized_buffer: List[str] = []

        # Last recognized utterance
        self._last_recognized: str = ""

        # Subscriber for recognized speech text from Whisper based ASR
        self._speech_sub = rospy.Subscriber(
            self._recognized_speech_topic,
            String,
            self._recognized_speech_cb,
        )

        # Publisher for text to speech
        self._tts_pub = rospy.Publisher(self._tts_topic, String, queue_size=10)

    def _recognized_speech_cb(self, msg: String) -> None:
        """Callback for recognized speech text."""
        text = msg.data if msg.data is not None else ""
        if not text:
            return

        self._last_recognized = text
        self._recognized_buffer.append(text)

    def clear_recognized_speech_buffer(self) -> None:
        """Clear internal buffer of recognized speech before a new dialog turn."""
        self._recognized_buffer.clear()

    def listen(self, timeout: float = 3.0) -> str:
        """
        Listen for an utterance assuming an speech recognition system is publishing
        recognized text on the configured topic.

        Returns the last recognized text within the given timeout.
        If nothing is recognized in that time, returns an empty string.
        """
        # If something is already in the buffer, return it immediately
        if self._recognized_buffer:
            return self._recognized_buffer[-1]

        start_time = rospy.Time.now()
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if self._recognized_buffer:
                return self._recognized_buffer[-1]

            if timeout is not None:
                elapsed = (rospy.Time.now() - start_time).to_sec()
                if elapsed >= timeout:
                    return ""

            rate.sleep()

        # Node is shutting down
        return ""

    def speak(self, text: str, blocking: bool = True) -> None:
        """
        Request text to speech for the given text.

        This method publishes the text on the configured TTS topic.
        The blocking flag is kept for API compatibility but there is
        no feedback channel here to wait for actual speech completion.
        """
        if not text:
            return

        msg = String()
        msg.data = text
        self._tts_pub.publish(msg)

        if blocking:
            rospy.logwarn(
                "HRI.speak called with blocking=True but no speech completion feedback is implemented. "
                "The call does not wait for playback to finish."
            )
