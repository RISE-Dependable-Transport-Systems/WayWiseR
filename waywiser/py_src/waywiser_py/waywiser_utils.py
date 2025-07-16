#!/usr/bin/env python3

import os
import yagmail

def get_full_file_path(file_path, package_relative_path=''):
    if file_path == '':
        return ''

    if file_path.startswith('~'):
        file_path = os.path.expanduser(file_path)
    file_path = os.path.expandvars(file_path)
    if not file_path.startswith('/') and package_relative_path != '':
        file_path = os.path.join(
            package_relative_path,
            file_path,
        )
    file_path = os.path.abspath(file_path)

    return file_path


def send_email(subject, body, email_recipient=None):
    """
    Send email using credentials from .env file

    Args:
        subject: Email subject
        body: Email body content
        email_recipient: Optional recipient (overrides .env default)
    """
    # Get credentials from .env
    email_user = os.getenv('EMAIL_USER')
    email_pass = os.getenv('EMAIL_PASSWORD')
    default_recipient = os.getenv('EMAIL_RECIPIENT')
    smtp_server = os.getenv('SMTP_SERVER', 'smtp.gmail.com')
    smtp_port = int(os.getenv('SMTP_PORT', 587))

    # Use provided recipient or fallback to .env default
    recipient = email_recipient if email_recipient is not None else default_recipient

    # Validate credentials
    if not all([email_user, email_pass, recipient]):
        print('Email credentials not found. Check your .env file.')
        return

    try:
        # Initialize SMTP connection
        yag = yagmail.SMTP(
            user=email_user,
            password=email_pass,
            host=smtp_server,
            port=smtp_port,
        )

        # Send email
        yag.send(to=recipient, subject=subject, contents=body)
        print('Email sent successfully!')
    except Exception as e:
        print(f'Error sending email: {e}')
