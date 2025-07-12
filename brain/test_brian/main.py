from dependency.load_dep import import_from_first_available
dep = import_from_first_available("dependency/depend.yaml")
move = dep["move"][1]["set_goal"]