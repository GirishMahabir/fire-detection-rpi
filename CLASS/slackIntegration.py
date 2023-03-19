import requests
import json
import toml


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

    def send(self):
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
                            "value": f"{ self.message }",
                            "short": "false"
                        }
                    ]
                }
            ]
        }
        return requests.post(self.webhook_url, json.dumps(payload))


if __name__ == '__main__':
    # Load config file
    config = toml.load('config.toml')
    # Create a slack message object
    slack_message = SlackMessage(
        config['slack']['webhook_url'], 'Fire Alert', 'danger')
    # Send the message
    slack_message.send()
