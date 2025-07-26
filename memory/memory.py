
from dataclasses import dataclass, field
from typing import List, Optional, Union
import numpy as np


# world model memory list
class WorldModelMemory:
    def __init__(self):
        # 记忆embeddings 的list
        self.memory_embeddings = []
        # 该记忆的子节点
        self.memory_child = []
        # 该记忆的类型
        self.memory_type = []

        

    def add(self, item):
        self.memory.append(item)

    def get(self, index):
        return self.memory[index] if 0 <= index < len(self.memory) else None

    def all(self):
        return self.memory

    def embedding(self):
        # Placeholder for embedding logic
        pass

    def search(self):
        # Placeholder for search logic
        pass

    def store(self):
        # Placeholder for store logic
        pass

    def retrieve(self):
        # Placeholder for retrieve logic
        pass


# short-term memory list
class ShortTermMemory:
    def __init__(self):
        self.memory = []

    def add(self, item):
        self.memory.append(item)

    def get(self, index):
        return self.memory[index] if 0 <= index < len(self.memory) else None

    def all(self):
        return self.memory

    def clear(self):
        self.memory.clear()

    def embedding(self):
        # Placeholder for embedding logic
        pass

    def search(self):
        # Placeholder for search logic
        pass

    def store(self):
        # Placeholder for store logic
        pass

    def retrieve(self):
        # Placeholder for retrieve logic
        pass







import logging
import networkx as nx  
from tqdm import tqdm
from huggingface_hub import HuggingFaceEmbeddings
from llm_interface import LLMInterface  # Assuming this is defined elsewhere
from spatial_relationship_extractor import SpatialRelationshipExtractor  # Assuming this is defined elsewhere





if __name__ == "__main__":
   from sentence_transformers import SentenceTransformer
    sentences = ["This is an example sentence", "Each sentence is converted"]

    model = SentenceTransformer('sentence-transformers/all-MiniLM-L6-v2')
    embeddings = model.encode(sentences)
    print(embeddings)




