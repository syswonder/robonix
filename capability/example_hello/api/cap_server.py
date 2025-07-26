#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import asyncio
from mcp.server.fastmcp import FastMCP



from hi_api import *
from hello2_api import *

if __name__ == "__main__":
    # init and run server
    mcp = FastMCP(name="demo_mult_api",
            host="127.0.0.1",
            port=8000,
            sse_path="/sse",
            message_path="/messages/")

    # init @mcp.tool()
    tools_list = []
    tools_list.append(get_hi_tools())
    tools_list.append(get_hello2_tools())

    # https://gofastmcp.com/servers/tools#decorator-arguments
    for tools in tools_list:
        for tool in tools:
            mcp.add_tool(
                fn=tool["fn"],
                name=tool["name"],
                description=tool["description"]
                # if contains other tools arguments
                # tag = tool["tag"] if "tag" in tool else None,
                # enabled = tool["enabled"] if "enabled" in tool else True
                # exclude_args = tool["exclude_args"] if "exclude_args" in tool else None,
                # annotations = tool["annotations"] if "annotations" in tool else None
            )
            
    # run server
    mcp.run(transport="sse")

    