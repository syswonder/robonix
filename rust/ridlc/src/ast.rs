// SPDX-License-Identifier: MulanPSL-2.0
// RIDL AST types (RFC001 subset)

/// Full RIDL file AST; multiple files merge by namespace.
#[derive(Default)]
pub struct File {
    pub namespace: Option<String>,
    pub imports: Vec<Import>,
    pub interfaces: Vec<Interface>,
}

#[derive(Clone, Debug)]
pub struct Import {
    pub path: String,       // e.g. "geometry_msgs/msg/Twist"
    pub wildcard: bool,     // import pkg/msg/*
}

#[derive(Clone, Debug)]
pub enum Interface {
    Stream(StreamDef),
    Command(CommandDef),
    Query(QueryDef),
    Event(EventDef),
}

#[derive(Clone, Debug)]
pub struct StreamDef {
    pub name: String,
    pub annotations: Vec<Annotation>,
    pub fields: Vec<StreamField>,
    pub version: Option<String>,
}

#[derive(Clone, Debug)]
pub enum StreamDirection {
    Input,
    Output,
}

#[derive(Clone, Debug)]
pub struct StreamField {
    pub direction: StreamDirection,
    pub name: String,
    pub type_ref: String,
    pub annotations: Vec<Annotation>,
}

#[derive(Clone, Debug)]
pub struct Annotation {
    pub key: String,
    pub value: Option<String>,
}

#[derive(Clone, Debug)]
pub struct CommandDef {
    pub name: String,
    pub annotations: Vec<Annotation>,
    pub input: Option<CommandField>,
    pub output: Option<CommandField>,
    pub result: Option<CommandField>,
    pub version: Option<String>,
    pub safety: Vec<SafetyItem>,
}

#[derive(Clone, Debug)]
pub struct CommandField {
    pub name: String,
    pub type_ref: String,
    pub annotations: Vec<Annotation>,
}

#[derive(Clone, Debug)]
pub struct SafetyItem {
    pub name: String,
    pub safety_type: String,
    pub rw: bool, // true = RW, false = RO
}

#[derive(Clone, Debug)]
pub struct QueryDef {
    pub name: String,
    pub annotations: Vec<Annotation>,
    pub request: QueryField,
    pub response: QueryField,
    pub version: Option<String>,
}

#[derive(Clone, Debug)]
pub struct QueryField {
    pub name: String,
    pub type_ref: String,
    pub annotations: Vec<Annotation>,
}

#[derive(Clone, Debug)]
pub struct EventDef {
    pub name: String,
    pub annotations: Vec<Annotation>,
    pub payload: EventPayload,
    pub version: Option<String>,
}

#[derive(Clone, Debug)]
pub struct EventPayload {
    pub name: String,
    pub type_ref: String,
    pub annotations: Vec<Annotation>,
}

impl File {
    pub fn merge(&mut self, other: File) {
        if self.namespace.is_none() && other.namespace.is_some() {
            self.namespace = other.namespace;
        }
        self.imports.extend(other.imports);
        self.interfaces.extend(other.interfaces);
    }

    pub fn namespace_path(&self) -> Option<&str> {
        self.namespace.as_deref()
    }
}
