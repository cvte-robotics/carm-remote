"""Construct the Quest input provider when the Board registry first needs it."""


def create_provider():
    from quest.module import QuestInputModule

    return QuestInputModule()
