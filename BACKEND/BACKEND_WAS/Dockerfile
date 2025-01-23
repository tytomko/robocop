FROM python:3.10.12 AS builder

ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1
WORKDIR /backend

RUN python -m venv .venv
COPY requirements.txt .
RUN .venv/bin/pip install -r requirements.txt

FROM python:3.10.12-slim
WORKDIR /backend
COPY --from=builder /backend/.venv .venv/
COPY ./app ./app
CMD ["python", "-m", "uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
