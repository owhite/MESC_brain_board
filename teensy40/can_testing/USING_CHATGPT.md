## Teensy Log Interpretation Workflow

1. Create an OpenAI account at [platform.openai.com](https://platform.openai.com).
2. Set up billing before using the API.
3. Add a payment method and purchase credits (or enable paid usage) so API calls do not fail with `insufficient_quota`.
4. Generate an API key from the **API Keys** page.
5. Copy the key once and store it securely.
6. Export the key in your terminal:
   ```bash
   export OPENAI_API_KEY="sk-..."
   ```
7. (Optional) Persist it in `~/.bashrc` or `~/.zshrc`, then restart terminal.
8. Install dependencies:
   ```bash
   python3 -m pip install --upgrade openai pyserial
   ```
9. Verify API connectivity:
   ```bash
   python baby_pi_test.py
   ```
10. Connect Teensy and identify the serial port (example: `/dev/ttyACM0`).
11. Run the interpreter:
   ```bash
   python interpret_balance_log.py --port /dev/ttyACM0
   ```
12. Press Enter when prompted; script sends `run` to Teensy.
13. Teensy runs and prints runtime JSON lines.
14. Script captures output until `balance exit` (or timeout), then sends logs to the API.
15. Interpretation is printed in the terminal (timing, motor health, CAN health, anomalies, verdict).
16. If API fails:
   - `insufficient_quota`: add credits/update billing.
   - `401/403`: check API key/project permissions.
   - `429`: retry later and check rate/quota limits.
