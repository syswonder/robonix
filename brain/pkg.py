import pkg_resources

def export_dependencies():
    try:
        # 获取所有已安装的包及其版本
        packages = [f"{dist.project_name}=={dist.version}" for dist in pkg_resources.working_set]
        
        # 写入 requirements.txt
        with open("requirements.txt", "w", encoding="utf-8") as f:
            f.write("\n".join(packages))
        
        print("依赖包已成功导出到 requirements.txt")
    
    except Exception as e:
        print(f"导出失败: {str(e)}")

if __name__ == "__main__":
    export_dependencies()