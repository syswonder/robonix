// SPDX-License-Identifier: MulanPSL-2.0
// Minimal RIDL parser (RFC001 subset)

use anyhow::{anyhow, Result};

use crate::ast::{
    Annotation, CommandDef, CommandField, EventDef, EventPayload, File, Import, Interface,
    QueryDef, QueryField, SafetyItem, StreamDef, StreamDirection, StreamField,
};

struct Lexer<'a> {
    s: &'a str,
    pos: usize,
}

impl<'a> Lexer<'a> {
    fn new(s: &'a str) -> Self {
        Lexer { s, pos: 0 }
    }
    fn line_col(&self, pos: usize) -> (usize, usize) {
        let mut line = 1;
        let mut col = 1;
        let end = pos.min(self.s.len());
        for ch in self.s[..end].chars() {
            if ch == '\n' {
                line += 1;
                col = 1;
            } else {
                col += 1;
            }
        }
        (line, col)
    }
    fn error(&self, msg: &str) -> anyhow::Error {
        let (line, col) = self.line_col(self.pos);
        anyhow!("{} at line {}, column {}", msg, line, col)
    }
    fn skip_ws_comments(&mut self) {
        loop {
            let rest = self.s[self.pos..].trim_start();
            self.pos = self.s.len() - rest.len();
            if rest.starts_with("//") {
                if let Some(nl) = rest.find('\n') {
                    self.pos += nl + 1;
                } else {
                    self.pos = self.s.len();
                }
            } else if rest.starts_with("/*") {
                if let Some(end) = rest.find("*/") {
                    self.pos += end + 2;
                } else {
                    self.pos = self.s.len();
                }
            } else {
                break;
            }
        }
    }
    fn peek_char(&self) -> Option<char> {
        self.s[self.pos..].chars().next()
    }
    fn next_char(&mut self) -> Option<char> {
        let c = self.peek_char()?;
        self.pos += c.len_utf8();
        Some(c)
    }
    fn take_ident(&mut self) -> Result<String> {
        self.skip_ws_comments();
        let start = self.pos;
        if let Some(c) = self.next_char() {
            if !c.is_alphabetic() && c != '_' {
                return Err(self.error("expected identifier"));
            }
        } else {
            return Err(self.error("unexpected eof"));
        }
        while self.peek_char().map(|c| c.is_alphanumeric() || c == '_').unwrap_or(false) {
            self.next_char();
        }
        Ok(self.s[start..self.pos].to_string())
    }
    fn take_type_ref(&mut self) -> Result<String> {
        self.skip_ws_comments();
        let start = self.pos;
        while let Some(c) = self.peek_char() {
            if c == ';' || c == '\n' || c == '{' || c == '}' || c == '@' {
                break;
            }
            self.next_char();
        }
        Ok(self.s[start..self.pos].trim().to_string())
    }
    fn expect_ident(&mut self, want: &str) -> Result<()> {
        let got = self.take_ident()?;
        if got != want {
            let msg = format!("expected '{}', got '{}'", want, got);
            return Err(self.error(&msg));
        }
        Ok(())
    }
    fn expect_char(&mut self, c: char) -> Result<()> {
        self.skip_ws_comments();
        match self.next_char() {
            Some(got) if got == c => Ok(()),
            _ => {
                let msg = format!("expected '{}'", c);
                Err(self.error(&msg))
            }
        }
    }
    /// Parse zero or more annotations: @key or @key("value"). Stops when no @ is found.
    fn parse_annotations(&mut self) -> Result<Vec<Annotation>> {
        let mut out = Vec::new();
        loop {
            self.skip_ws_comments();
            if self.peek_char() != Some('@') {
                break;
            }
            self.next_char(); // consume '@'
            let key = self.take_ident()?;
            self.skip_ws_comments();
            let value = if self.peek_char() == Some('(') {
                self.next_char(); // consume '('
                self.skip_ws_comments();
                let v = if self.peek_char() == Some('"') {
                    self.next_char(); // consume '"'
                    let start = self.pos;
                    while let Some(c) = self.peek_char() {
                        if c == '"' {
                            break;
                        }
                        self.next_char();
                    }
                    let s = self.s[start..self.pos].to_string();
                    if self.peek_char() == Some('"') {
                        self.next_char();
                    }
                    s
                } else {
                    let start = self.pos;
                    while self.peek_char().map(|c| c.is_alphanumeric() || c == '_' || c == '=' || c.is_ascii_digit() || c == '.').unwrap_or(false) {
                        self.next_char();
                    }
                    self.s[start..self.pos].trim().to_string()
                };
                self.skip_ws_comments();
                if self.peek_char() == Some(')') {
                    self.next_char();
                }
                Some(v)
            } else {
                None
            };
            out.push(Annotation { key, value });
        }
        Ok(out)
    }
    fn parse_namespace_decl(&mut self) -> Result<String> {
        self.expect_ident("namespace")?;
        self.skip_ws_comments();
        let start = self.pos;
        while self.peek_char().map(|c| c.is_alphanumeric() || c == '_' || c == '/').unwrap_or(false) {
            self.next_char();
        }
        let path = self.s[start..self.pos].trim().to_string();
        if path.is_empty() {
            return Err(self.error("empty namespace path"));
        }
        Ok(path)
    }
    fn parse_import_decl(&mut self) -> Result<Import> {
        self.expect_ident("import")?;
        self.skip_ws_comments();
        let start = self.pos;
        while self
            .peek_char()
            .map(|c| c.is_alphanumeric() || c == '_' || c == '/' || c == '.')
            .unwrap_or(false)
        {
            self.next_char();
        }
        let mut path = self.s[start..self.pos].trim().to_string();
        self.skip_ws_comments();
        let wildcard = self.s[self.pos..].starts_with('/')
            && self.s[self.pos + 1..].chars().next() == Some('*');
        if wildcard {
            self.pos += 2;
            path = path.trim_end_matches('/').to_string();
        }
        Ok(Import { path, wildcard })
    }
    fn parse_query_def(&mut self) -> Result<QueryDef> {
        self.expect_ident("query")?;
        let name = self.take_ident()?;
        let annotations = self.parse_annotations()?;
        self.expect_char('{')?;
        let mut request = None;
        let mut response = None;
        let mut version = None;
        loop {
            self.skip_ws_comments();
            // End of query body.
            if matches!(self.peek_char(), Some('}')) {
                break;
            }
            let kw = self.take_ident();
            if kw.is_err() {
                break;
            }
            let kw = kw.unwrap();
            match kw.as_str() {
                "request" => {
                    let fname = self.take_ident()?;
                    let type_ref = self.take_type_ref()?;
                    let field_ann = self.parse_annotations()?;
                    self.expect_char(';')?;
                    request = Some(QueryField {
                        name: fname,
                        type_ref,
                        annotations: field_ann,
                    });
                }
                "response" => {
                    let fname = self.take_ident()?;
                    let type_ref = self.take_type_ref()?;
                    let field_ann = self.parse_annotations()?;
                    self.expect_char(';')?;
                    response = Some(QueryField {
                        name: fname,
                        type_ref,
                        annotations: field_ann,
                    });
                }
                "version" => {
                    self.skip_ws_comments();
                    let start = self.pos;
                    while self
                        .peek_char()
                        .map(|c| c.is_ascii_digit() || c == '.')
                        .unwrap_or(false)
                    {
                        self.next_char();
                    }
                    version = Some(self.s[start..self.pos].to_string());
                    self.expect_char(';')?;
                }
                _ => break,
            }
        }
        self.expect_char('}')?;
        Ok(QueryDef {
            name,
            annotations,
            request: request.ok_or_else(|| anyhow!("query must have request"))?,
            response: response.ok_or_else(|| anyhow!("query must have response"))?,
            version,
        })
    }
    fn parse_stream_def(&mut self) -> Result<StreamDef> {
        self.expect_ident("stream")?;
        let name = self.take_ident()?;
        let annotations = self.parse_annotations()?;
        self.expect_char('{')?;
        let mut fields = Vec::new();
        let mut version = None;
        loop {
            self.skip_ws_comments();
            // If next non-whitespace is '}', we've reached end of stream body.
            if matches!(self.peek_char(), Some('}')) {
                break;
            }
            let kw = self.take_ident();
            if kw.is_err() {
                break;
            }
            let kw = kw.unwrap();
            if kw == "output" || kw == "input" {
                let fname = self.take_ident()?;
                let type_ref = self.take_type_ref()?;
                let field_ann = self.parse_annotations()?;
                // Semicolon at end of line is optional; allow newline+closing brace.
                self.skip_ws_comments();
                if let Some(';') = self.peek_char() {
                    self.next_char();
                }
                fields.push(StreamField {
                    direction: if kw == "input" {
                        StreamDirection::Input
                    } else {
                        StreamDirection::Output
                    },
                    name: fname,
                    type_ref,
                    annotations: field_ann,
                });
            } else if kw == "version" {
                self.skip_ws_comments();
                let start = self.pos;
                while self
                    .peek_char()
                    .map(|c| c.is_ascii_digit() || c == '.')
                    .unwrap_or(false)
                {
                    self.next_char();
                }
                version = Some(self.s[start..self.pos].to_string());
                self.expect_char(';')?;
            } else {
                break;
            }
        }
        self.expect_char('}')?;
        Ok(StreamDef {
            name,
            annotations,
            fields,
            version,
        })
    }
    fn parse_command_def(&mut self) -> Result<CommandDef> {
        self.expect_ident("command")?;
        let name = self.take_ident()?;
        let annotations = self.parse_annotations()?;
        self.expect_char('{')?;
        let mut input = None;
        let mut output = None;
        let mut result = None;
        let mut version = None;
        let mut safety = Vec::new();
        loop {
            self.skip_ws_comments();
            // End of command body.
            if matches!(self.peek_char(), Some('}')) {
                break;
            }
            let kw = self.take_ident();
            if kw.is_err() {
                break;
            }
            let kw = kw.unwrap();
            match kw.as_str() {
                "input" => {
                    let fname = self.take_ident()?;
                    let type_ref = self.take_type_ref()?;
                    let field_ann = self.parse_annotations()?;
                    self.skip_ws_comments();
                    if let Some(';') = self.peek_char() {
                        self.next_char();
                    }
                    input = Some(CommandField {
                        name: fname,
                        type_ref,
                        annotations: field_ann,
                    });
                }
                "output" => {
                    let fname = self.take_ident()?;
                    let type_ref = self.take_type_ref()?;
                    let field_ann = self.parse_annotations()?;
                    self.skip_ws_comments();
                    if let Some(';') = self.peek_char() {
                        self.next_char();
                    }
                    output = Some(CommandField {
                        name: fname,
                        type_ref,
                        annotations: field_ann,
                    });
                }
                "result" => {
                    let fname = self.take_ident()?;
                    let type_ref = self.take_type_ref()?;
                    let field_ann = self.parse_annotations()?;
                    self.skip_ws_comments();
                    if let Some(';') = self.peek_char() {
                        self.next_char();
                    }
                    result = Some(CommandField {
                        name: fname,
                        type_ref,
                        annotations: field_ann,
                    });
                }
                "version" => {
                    self.skip_ws_comments();
                    let start = self.pos;
                    while self
                        .peek_char()
                        .map(|c| c.is_ascii_digit() || c == '.')
                        .unwrap_or(false)
                    {
                        self.next_char();
                    }
                    version = Some(self.s[start..self.pos].to_string());
                    self.expect_char(';')?;
                }
                "safety" => {
                    self.expect_char('{')?;
                    loop {
                        self.skip_ws_comments();
                        // End of safety block.
                        if matches!(self.peek_char(), Some('}')) {
                            break;
                        }
                        let sname = self.take_ident();
                        if sname.is_err() {
                            break;
                        }
                        let sname = sname.unwrap();
                        self.skip_ws_comments();
                        // Parse safety type and RO/RW access, allowing either:
                        //   name type RW
                        // or
                        //   name type
                        //   RW
                        let start = self.pos;
                        while let Some(c) = self.peek_char() {
                            if c == '\n' || c == '}' {
                                break;
                            }
                            self.next_char();
                        }
                        let line = self.s[start..self.pos].trim().to_string();
                        if line.is_empty() {
                            return Err(self.error("expected safety type"));
                        }
                        let mut parts: Vec<&str> = line.split_whitespace().collect();
                        let mut stype = String::new();
                        let mut ro_rw = String::new();
                        if let Some(last) = parts.last().cloned() {
                            if last == "RO" || last == "RW" {
                                ro_rw = last.to_string();
                                parts.pop();
                                stype = parts.join(" ");
                            }
                        }
                        if stype.is_empty() {
                            // Fallback to old style where RO/RW is on the next line.
                            stype = line;
                            self.skip_ws_comments();
                            ro_rw = self.take_ident()?;
                        }
                        // Semicolon at end of safety item is optional.
                        self.skip_ws_comments();
                        if let Some(';') = self.peek_char() {
                            self.next_char();
                        }
                        safety.push(SafetyItem {
                            name: sname,
                            safety_type: stype,
                            rw: ro_rw == "RW",
                        });
                    }
                    self.expect_char('}')?;
                }
                _ => break,
            }
        }
        self.expect_char('}')?;
        Ok(CommandDef {
            name,
            annotations,
            input,
            output,
            result,
            version,
            safety,
        })
    }
    fn parse_event_def(&mut self) -> Result<EventDef> {
        self.expect_ident("event")?;
        let name = self.take_ident()?;
        let annotations = self.parse_annotations()?;
        self.expect_char('{')?;
        let mut payload = None;
        let mut version = None;
        loop {
            self.skip_ws_comments();
            // End of event body.
            if matches!(self.peek_char(), Some('}')) {
                break;
            }
            let kw = self.take_ident();
            if kw.is_err() {
                break;
            }
            let kw = kw.unwrap();
            if kw == "payload" {
                let pname = self.take_ident()?;
                let type_ref = self.take_type_ref()?;
                let field_ann = self.parse_annotations()?;
                self.skip_ws_comments();
                if let Some(';') = self.peek_char() {
                    self.next_char();
                }
                payload = Some(EventPayload {
                    name: pname,
                    type_ref,
                    annotations: field_ann,
                });
            } else if kw == "version" {
                self.skip_ws_comments();
                let start = self.pos;
                while self
                    .peek_char()
                    .map(|c| c.is_ascii_digit() || c == '.')
                    .unwrap_or(false)
                {
                    self.next_char();
                }
                version = Some(self.s[start..self.pos].to_string());
                self.expect_char(';')?;
            } else {
                break;
            }
        }
        self.expect_char('}')?;
        Ok(EventDef {
            name,
            annotations,
            payload: payload.ok_or_else(|| anyhow!("event must have payload"))?,
            version,
        })
    }
}

pub fn parse_file(content: &str) -> Result<File> {
    let mut lex = Lexer::new(content);
    let mut file = File::default();

    lex.skip_ws_comments();
    while lex.pos < content.len() {
        let rest = content[lex.pos..].trim_start();
        lex.pos = content.len() - rest.len();
        if rest.is_empty() {
            break;
        }
        if rest.starts_with("namespace") {
            file.namespace = Some(lex.parse_namespace_decl()?);
        } else if rest.starts_with("import") {
            file.imports.push(lex.parse_import_decl()?);
        } else if rest.starts_with("query") {
            file.interfaces
                .push(Interface::Query(lex.parse_query_def()?));
        } else if rest.starts_with("stream") {
            file.interfaces
                .push(Interface::Stream(lex.parse_stream_def()?));
        } else if rest.starts_with("command") {
            file.interfaces
                .push(Interface::Command(lex.parse_command_def()?));
        } else if rest.starts_with("event") {
            file.interfaces
                .push(Interface::Event(lex.parse_event_def()?));
        } else {
            lex.next_char().ok_or_else(|| lex.error("unexpected token"))?;
        }
        lex.skip_ws_comments();
    }
    Ok(file)
}
