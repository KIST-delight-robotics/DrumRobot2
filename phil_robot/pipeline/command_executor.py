import time


def execute_commands(bot, commands):
    """
    검증이 끝난 명령만 실제 로봇으로 보낸다.
    wait 명령은 로봇 네이티브 지원 전까지 Python executor가 임시로 담당한다.
    """
    executed_commands = []

    for command in commands:
        if command.startswith("wait:"):
            _, _, seconds_raw = command.partition(":")
            seconds = float(seconds_raw)
            print(f"⏳ 실행 대기: {seconds:.2f}초")
            time.sleep(seconds)
            executed_commands.append(command)
            continue

        print(f"📡 명령 전송: {command}")
        if bot.send_command(command + "\n"):
            executed_commands.append(command)

    return executed_commands
