# import requests
# import json
# import toml


# class SlackMessage:
#     """
#     Class to send a slack notification.
#     """

#     def __init__(self, webhook_url, message, level):
#         """
#         :param webhook_url:(str): Slack Webhook URL.
#         :param message:(str): Slack Message.
#         :param level:(str): good, warning or danger.
#         """
#         self.webhook_url = webhook_url
#         self.message = message
#         self.level = level

#     def send(self, message=None, level=None):
#         self.message = message if message else self.message
#         self.level = level if level else self.level
#         payload = {
#             "attachments": [
#                 {
#                     "fallback": "Alert System",
#                     "text": "Alert System Summary",
#                     "pretext": "Alert System Summary",
#                     "color": self.level,  # 'good', 'warning', 'danger'
#                     "fields": [
#                         {
#                             "title": "Fire Alert",
#                             "value": f"{ self.message }",
#                             "short": "false"
#                         }
#                     ]
#                 }
#             ]
#         }
#         return requests.post(self.webhook_url, json.dumps(payload))

# if __name__ == '__main__':
#     # Load config file
#     config = toml.load('config.toml')
#     # Create a slack message object
#     slack_message = SlackMessage(
#         config['slack']['webhook_url'], 'Fire Alert', 'danger')
#     # Send the message
#     slack_message.send()
import requests
import json
import toml
import base64


class SlackMessage:
    """
    Class to send a slack notification.
    """

    def __init__(self, webhook_url, message, level):
        """
        :param webhook_url:(str): Slack Webhook URL.
        :param message:(str): Slack Message.
        :param level:(str): good, warning or danger.
        """
        self.webhook_url = webhook_url
        self.message = message
        self.level = level

    def send(self, message=None, level=None, frame=None):
        self.message = message if message else self.message
        self.level = level if level else self.level

        payload = {
            "attachments": [
                {
                    "fallback": "Alert System",
                    "text": "Alert System Summary",
                    "pretext": "Alert System Summary",
                    "color": self.level,  # 'good', 'warning', 'danger'
                    "fields": [
                        {
                            "title": "Fire Alert",
                            "value": f"{self.message}",
                            "short": "false"
                        }
                    ]
                }
            ]
        }

        if frame is not None:
            # Convert the frame to base64 encoding
            frame_base64 = base64.b64encode(frame).decode('utf-8')
            payload['attachments'][0]['image_url'] = f"data:image/jpeg;base64,{frame_base64}"

        return requests.post(self.webhook_url, json.dumps(payload))


if __name__ == '__main__':
    # Load config file
    config = toml.load('config.toml')
    # Create a slack message object
    slack_message = SlackMessage(
        config['slack']['webhook_url'], 'Fire Alert', 'danger')
    # Capture the frame using CV2 (replace this with your CV2 code)
    frame = cv2.imread('frame.jpg')  # Replace 'frame.jpg' with the actual path to the frame
    # Send the message with the frame
    slack_message.send(frame=frame)
