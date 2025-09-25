# robonix 文档系统

wheatfox

本目录包含 robonix 项目的完整技术文档，使用 Sphinx 构建系统生成。

## 构建文档

### 安装依赖

```bash
pip install sphinx sphinxawesome-theme myst-parser
```

### 构建 HTML 文档

```bash
cd docs
make html
```

构建完成后，HTML 文档位于 `build/html/index.html`。

### 其他格式

```bash
# 构建 PDF 文档（需要 LaTeX）
make latexpdf

# 构建 EPUB 文档
make epub

# 清理构建文件
make clean
```

## 维护说明

- 文档源文件使用 reStructuredText 格式
- 配置文件 `conf.py` 包含了主题、扩展和路径设置
- API 文档通过 `autodoc` 扩展自动生成
- 支持 Markdown 文件（通过 `myst-parser` 扩展）

## 贡献指南

1. 修改 `source/` 目录下的 `.rst` 文件
2. 运行 `make html` 验证构建结果
3. 确保文档内容准确、清晰、专业
4. 遵循现有的文档结构和风格约定