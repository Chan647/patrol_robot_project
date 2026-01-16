import boto3
import uuid
import cv2
from botocore.exceptions import ClientError

s3 = boto3.client(
    's3',
    endpoint_url='http://192.168.0.17:9000',
    aws_access_key_id='minioadmin',
    aws_secret_access_key='minioadmin',
    region_name='us-east-1'
)

BUCKET_NAME = 'patrolimage'


def upload_image_to_minio(frame):
    filename = f"{uuid.uuid4()}.png"

    success, buffer = cv2.imencode('.png', frame)
    if not success:
        return None

    try:
        try:
            s3.head_bucket(Bucket=BUCKET_NAME)
        except ClientError:
            s3.create_bucket(Bucket=BUCKET_NAME)

        s3.put_object(
            Bucket=BUCKET_NAME,
            Key=filename,
            Body=buffer.tobytes(),
            ContentType='image/png'
        )

        return f"http://192.168.0.17:9000/{BUCKET_NAME}/{filename}"

    except Exception as e:
        print(f"[MinIO Upload Error] {e}")
        return None
