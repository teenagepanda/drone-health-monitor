import os
import smtplib
from email.message import EmailMessage
from pathlib import Path
from dotenv import load_dotenv


def send_email_report(subject: str, body: str, attachments: list[str]) -> None:
    load_dotenv()

    email_from = os.getenv("EMAIL_FROM")
    email_password = os.getenv("EMAIL_PASSWORD")
    email_to = os.getenv("EMAIL_TO")
    smtp_server = os.getenv("SMTP_SERVER", "smtp.gmail.com")
    smtp_port = int(os.getenv("SMTP_PORT", "587"))

    missing = []
    for name, value in {
        "EMAIL_FROM": email_from,
        "EMAIL_PASSWORD": email_password,
        "EMAIL_TO": email_to,
    }.items():
        if not value:
            missing.append(name)

    if missing:
        raise RuntimeError(f"Missing email configuration in .env: {', '.join(missing)}")

    msg = EmailMessage()
    msg["Subject"] = subject
    msg["From"] = email_from
    msg["To"] = email_to
    msg.set_content(body)

    for attachment in attachments:
        path = Path(attachment)
        if not path.exists():
            continue

        data = path.read_bytes()
        if path.suffix.lower() == ".csv":
            maintype, subtype = "text", "csv"
        elif path.suffix.lower() == ".html":
            maintype, subtype = "text", "html"
        else:
            maintype, subtype = "application", "octet-stream"

        msg.add_attachment(
            data,
            maintype=maintype,
            subtype=subtype,
            filename=path.name
        )

    with smtplib.SMTP(smtp_server, smtp_port) as smtp:
        smtp.starttls()
        smtp.login(email_from, email_password)
        smtp.send_message(msg)
