# CCS MCP bridge troubleshooting

Use this flow for CCStudio IDE Project, SysConfig, Debug, or Serial MCP failures.

## Lifecycle facts

- CCStudio IDE MCP servers are local STDIO proxies. They require CCStudio IDE to be running with the intended workspace active and `Enable CCStudio IDE ecosystem MCP servers` applied in the AI Assistant Configurator.
- Closing CCStudio stops its MCP backends. Reopening CCStudio may still require restarting the Codex extension/app task so the client reloads its tool registry.
- The four configured proxies normally use the CCStudio-bundled Node executable and `mcp-server-proxy.js` with arguments `debug`, `project`, `sysconfig`, and `serial`.
- Internet access is not required for the local Codex-to-CCStudio bridge itself. Keep provider/network failures separate from local MCP lifecycle failures.

Official references:

- [TI CCStudio IDE AI Coding Assistants](https://software-dl.ti.com/ccs/esd/documents/users_guide/ccs_ai.html)
- [Codex Model Context Protocol](https://learn.chatgpt.com/docs/extend/mcp)

## Diagnose by layer

1. Read `{ccs-install-dir}/ccs/theia/resources/ai/CCS.md` before calling any CCS MCP tool.
2. Inspect the current tool registry for `mcp__ccs_project__*`, `mcp__ccs_sysconfig__*`, `mcp__ccs_debug__*`, and `mcp__ccs_serial__*`. Do not assume an old task's tool snapshot is current.
3. Run `codex mcp list`, then `codex mcp get <server>` for each CCS server. Confirm the services are enabled and their Node/proxy paths point at the active CCStudio installation. `Auth Unsupported` is not a failure for these local STDIO servers.
4. Confirm CCStudio and its bundled Node processes are running. Do not manually launch a proxy as a substitute for an inactive CCStudio workspace.
5. Test the least invasive real calls:
   - Project: get the active project or project descriptors.
   - SysConfig: list files, open the active `.syscfg`, then read diagnostics.
   - Debug: list connected devices.
   - Serial: list serial ports and connected ports.
6. Classify results correctly. An empty device or serial-port list means the MCP server responded but no corresponding hardware/port was detected; it is not an MCP connection failure.
7. Inspect recent MCP logs only after the live health calls. A `resources/list` `-32601 Method not found` message is not the blocker when tool discovery and actual tool calls succeed.

## Recovery order

1. Keep CCStudio IDE open on the correct workspace.
2. In AI Assistant Configurator, enable the CCStudio ecosystem MCP servers and press **Apply**.
3. Restart the Codex extension/app or start a new task so its MCP tool list is rebuilt.
4. Re-run `codex mcp list` and the live health calls above.
5. Reapply the AI Assistant Configurator settings and restart CCStudio only if the configuration or live calls remain wrong.
6. Reboot the computer only after the narrower restart sequence fails.

Do not rewrite a correct MCP configuration, increase startup timeouts, reinstall CCStudio, hand-edit `.syscfg`, or invoke CCS build tools outside the Project MCP server without evidence that the corresponding layer is faulty.
