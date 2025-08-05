class SkillProvider:
    def __init__(self, name: str, IP: str, skills: list[str]):
        self.name: str = name
        self.IP: str = IP
        self.skills: list[str] = skills

    def __str__(self):
        return f"SkillProvider(name={self.name}, IP={self.IP}, skills={self.skills})"

    def __repr__(self):
        return self.__str__()
