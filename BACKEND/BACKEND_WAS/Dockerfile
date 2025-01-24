FROM python:3.10.12 AS builder

ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1
WORKDIR /BACKEND_WAS

RUN python -m venv .venv
COPY requirements.txt .
RUN .venv/bin/pip install -r requirements.txt

FROM python:3.10.12-slim
WORKDIR /BACKEND_WAS
COPY --from=builder /BACKEND_WAS/.venv .venv/
COPY ./app ./app
CMD [".venv/bin/python", "-m", "uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
