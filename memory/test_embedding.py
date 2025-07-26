'''
from sentence_transformers import SentenceTransformer
sentences = ["This is an example sentence", "Each sentence is converted"]

model = SentenceTransformer('./all-MiniLM-L6-v2')
embeddings = model.encode(sentences)
# 打印长度
print(len(embeddings))  # 输出嵌入向量的长度
decoded_embeddings = model.decode(embeddings) # 不行
'''

'''
from transformers import AutoTokenizer, AutoModel
import torch
import torch.nn.functional as F

# Mean Pooling function takes care of the correct averaging
def mean_pooling(model_output, attention_mask):
    token_embeddings = model_output[0]  # Get token embeddings
    input_mask_expanded = attention_mask.unsqueeze(-1).expand(token_embeddings.size()).float()
    return torch.sum(token_embeddings * input_mask_expanded, 1) / torch.clamp(input_mask_expanded.sum(1), min=1e-9)

sentences = ["This is an example sentence", "Each sentence is converted"]
tokenizer = AutoTokenizer.from_pretrained('./all-MiniLM-L6-v2')
model = AutoModel.from_pretrained('./all-MiniLM-L6-v2')

# Tokenize sentences
encoded_input = tokenizer(sentences, padding=True, truncation=True, return_tensors='pt')

# Compute token embeddings
with torch.no_grad():
    model_output = model(**encoded_input)

# Perform pooling to get sentence embeddings
sentence_embeddings = mean_pooling(model_output, encoded_input['attention_mask'])

# Normalizing embeddings
sentence_embeddings = F.normalize(sentence_embeddings, p=2, dim=1)

print("Sentence embeddings:")
print(sentence_embeddings)
'''

from transformers import AutoModel, AutoTokenizer

import torch
import numpy as np

# 指定模型名称
model_name = "./all-MiniLM-L6-v2"

# 加载分词器和模型
tokenizer = AutoTokenizer.from_pretrained(model_name)
model = AutoModel.from_pretrained(model_name)

sentences = ["餐厅有杯子和水", "我需要再接杯水"]

# 编码文本
inputs = tokenizer(sentences, padding=True, truncation=True, return_tensors="pt")
with torch.no_grad():
    outputs = model(**inputs)

# 提取句嵌入（均值池化）
embeddings = outputs.last_hidden_state.mean(dim=1).numpy()

# 打印嵌入向量len
# print(f"嵌入向量长度: {len(embeddings[0])}")  # 输出嵌入向量的长度
# 计算余弦相似度
similarity = np.dot(embeddings[0], embeddings[1]) / (
    np.linalg.norm(embeddings[0]) * np.linalg.norm(embeddings[1])
)
print(f"对比句子为: {sentences[0]} 和 {sentences[1]}")
print(f"相似度: {similarity:.4f}")  # 示例输出: 相似度: 0.5033
 


