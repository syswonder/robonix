class SkillProvider:
    def __init__(self, name: str, IP: str, skills: list[str], port: int = None):
        self.name: str = name
        self.IP: str = IP
        self.port: int = port
        self.skills: list[str] = skills

    def __str__(self):
        return f"SkillProvider(name={self.name}, IP={self.IP}, port={self.port}, skills={self.skills})"

    def __repr__(self):
        return self.__str__()
    
    def dump_skills(self):
        # dump beatified skills list in color
        if not self.skills:
            print("\033[93m(no skills)\033[0m")
            return None
            
        print("\033[92m[", end="")
        for i, skill in enumerate(self.skills):
            if i == len(self.skills) - 1:
                print(f"{skill}", end="")
            else:
                print(f"{skill}, ", end="")
        print("]\033[0m")
        return None  # Explicitly return None since this is a print-only method