// SPDX-License-Identifier: MulanPSL-2.0
//! Rust Channel bindings reuse the resolved protobuf method types.

use super::{
    ContractToml, ResolvedType, contract_id_to_service_name, contract_method_name, parse_idl_path,
    upper_camel,
};
use crate::codegen::msg_parser::{MsgResolver, MsgSpec, MsgTypeRef};
use anyhow::{Context, Result, ensure};
use heck::ToUpperCamelCase;
use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write;
use std::path::{Path, PathBuf};

fn marker(contract: &ContractToml) -> String {
    let service = contract_id_to_service_name(&contract.contract.id);
    service
        .strip_prefix("Robonix")
        .unwrap_or(&service)
        .to_owned()
}

fn package(contract: &ContractToml) -> Result<&str> {
    parse_idl_path(contract.contract.idl.trim())
        .map(|(package, _, _)| package)
        .context("missing contract IDL package")
}

fn rust_type(value: &ResolvedType) -> String {
    match value {
        ResolvedType::GoogleEmpty => "()".to_owned(),
        ResolvedType::StringWire => "super::contracts::StringWire".to_owned(),
        ResolvedType::ProtoFqn(name) => {
            let (module, name) = name.rsplit_once('.').unwrap();
            let module = module.rsplit('.').next().unwrap();
            format!("super::{module}::{}", name.to_upper_camel_case())
        }
    }
}

fn stream(value: &str) -> String {
    format!("std::pin::Pin<Box<dyn tokio_stream::Stream<Item = {value}> + Send>>")
}

fn response<'a>(contract: &ContractToml, resolver: &'a MsgResolver) -> Result<&'a MsgSpec> {
    let (package, _, name) =
        parse_idl_path(contract.contract.idl.trim()).context("missing service IDL")?;
    Ok(&resolver
        .srv_spec(package, name)
        .context("missing resolved service")?
        .response)
}

fn has_field(spec: &MsgSpec, name: &str, primitive: &str) -> Result<bool> {
    let Some(field) = spec.fields.iter().find(|field| field.name == name) else {
        return Ok(false);
    };
    ensure!(
        !field.is_array
            && matches!(&field.type_ref, MsgTypeRef::Primitive(kind) if kind == primitive),
        "async field {name} must have type {primitive}"
    );
    Ok(true)
}

fn request(contract: &ContractToml, ty: &str, resolver: &MsgResolver) -> Result<String> {
    let (package, _, name) =
        parse_idl_path(contract.contract.idl.trim()).context("missing service IDL")?;
    let spec = &resolver
        .srv_spec(package, name)
        .context("missing resolved service")?
        .request;
    if has_field(spec, "run_id", "string")? {
        let defaults = if spec.fields.len() == 1 {
            ""
        } else {
            ", ..Default::default()"
        };
        Ok(format!("Ok({ty} {{ run_id: _run_id{defaults} }})"))
    } else {
        ensure!(
            spec.fields.is_empty(),
            "async companion request requires run_id or an empty request"
        );
        Ok("{ anyhow::ensure!(_run_id.is_empty(), \"empty companion request cannot address a confirmed run_id\"); Ok(Default::default()) }".to_owned())
    }
}

pub(super) fn generate(
    contracts: &[(PathBuf, ContractToml)],
    types: &[(String, ResolvedType, ResolvedType)],
    resolver: &MsgResolver,
    out_dir: &Path,
    verbose: bool,
) -> Result<()> {
    let indices: BTreeMap<_, _> = contracts
        .iter()
        .enumerate()
        .map(|(index, (_, contract))| (contract.contract.id.as_str(), index))
        .collect();
    let mut modules = BTreeMap::<String, String>::new();
    let mut names = BTreeSet::new();
    for (index, (_, contract)) in contracts.iter().enumerate() {
        let id = &contract.contract.id;
        let package = package(contract)?;
        let name = marker(contract);
        ensure!(
            names.insert((package, name.clone())),
            "duplicate Channel binding: {package}::{name}"
        );
        let out = modules.entry(package.to_owned()).or_default();
        let service = contract_id_to_service_name(id);
        let (_, kind, idl_name) = parse_idl_path(contract.contract.idl.trim()).unwrap();
        let method = contract_method_name(contract);
        let mode = contract.mode.mode_type.trim();
        let input = rust_type(&types[index].1);
        let output = rust_type(&types[index].2);
        let (function, call_input, call_output) = match mode {
            "rpc" => ("unary", input.clone(), output.clone()),
            "rpc_server_stream" | "topic_out" => (
                "server_stream",
                input.clone(),
                stream(&format!("anyhow::Result<{output}>")),
            ),
            "rpc_client_stream" | "topic_in" => ("client_stream", stream(&input), output.clone()),
            "rpc_bidirectional_stream" => (
                "bidirectional_stream",
                stream(&input),
                stream(&format!("anyhow::Result<{output}>")),
            ),
            _ => unreachable!(),
        };
        let companions = if mode == "rpc" {
            indices
                .get(format!("{id}/status").as_str())
                .zip(indices.get(format!("{id}/cancel").as_str()))
        } else {
            None
        };
        let (status_type, cancel_type) = companions
            .map(|(status, cancel)| (rust_type(&types[*status].2), rust_type(&types[*cancel].2)))
            .unwrap_or_else(|| ("()".to_owned(), "()".to_owned()));
        writeln!(
            out,
            "/// Typed communication for `{id}`.\n#[derive(Clone)]\npub struct {name};"
        )?;
        writeln!(
            out,
            "impl robonix_atlas::client::channel::ChannelContract for {name} {{"
        )?;
        writeln!(
            out,
            "type Input = {call_input};\ntype Output = {call_output};\ntype StatusResponse = {status_type};\ntype CancelResponse = {cancel_type};"
        )?;
        writeln!(
            out,
            "const TRANSPORT: robonix_atlas::pb::Transport = robonix_atlas::pb::Transport::Grpc;"
        )?;
        for (constant, value) in [
            ("ID", id.as_str()),
            ("MODE", mode),
            ("SERVICE", &format!("robonix.contracts.{service}")),
            ("METHOD", &method),
            ("IDL", &format!("{package}/{kind}/{idl_name}")),
        ] {
            writeln!(out, "const {constant}: &'static str = {value:?};")?;
        }
        if id == "robonix/lifecycle/driver" {
            writeln!(
                out,
                "fn matches_contract(contract: &robonix_atlas::pb::ContractDescriptor) -> bool {{ contract.id == Self::ID || (contract.id.ends_with(\"/driver\") && contract.io_srv_type == Self::IDL) }}"
            )?;
        }
        writeln!(
            out,
            "async fn invoke(channel: robonix_atlas::client::channel::Channel<Self>, request: Self::Input) -> anyhow::Result<Self::Output> {{\nrobonix_atlas::client::channel::grpc::{function}::<Self, {input}, {output}>(channel, request).await\n}}"
        )?;
        if let Some((status, cancel)) = companions {
            let spec = response(contract, resolver)?;
            let accepted = if has_field(spec, "accepted", "bool")? {
                "_response.accepted"
            } else {
                "true"
            };
            let run_id = if has_field(spec, "run_id", "string")? {
                "_response.run_id.clone()"
            } else {
                "String::new()"
            };
            writeln!(
                out,
                "fn execution(_response: &Self::Output) -> anyhow::Result<(bool, String)> {{ Ok(({accepted}, {run_id})) }}"
            )?;
            for (is_status, method_name, companion_index) in
                [(true, "status", *status), (false, "cancel", *cancel)]
            {
                let companion = &contracts[companion_index].1;
                ensure!(
                    companion.mode.mode_type.trim() == "rpc",
                    "async companion must use rpc mode"
                );
                let binding = format!(
                    "super::{}::{}",
                    self::package(companion)?,
                    marker(companion)
                );
                let req = request(companion, &rust_type(&types[companion_index].1), resolver)?;
                writeln!(
                    out,
                    "async fn {method_name}(channel: robonix_atlas::client::channel::Channel<Self>) -> anyhow::Result<Self::{}Response> {{ channel.companion_call::<{binding}>({is_status}, |_run_id| {req}).await }}",
                    upper_camel(method_name)
                )?;
            }
            let status_binding = format!(
                "super::{}::{}",
                self::package(&contracts[*status].1)?,
                marker(&contracts[*status].1)
            );
            let cancel_binding = format!(
                "super::{}::{}",
                self::package(&contracts[*cancel].1)?,
                marker(&contracts[*cancel].1)
            );
            writeln!(
                out,
                "fn validate_companion(contract: &robonix_atlas::pb::ContractDescriptor, status: bool) -> anyhow::Result<()> {{ if status {{ robonix_atlas::client::channel::validate_contract::<{status_binding}>(contract) }} else {{ robonix_atlas::client::channel::validate_contract::<{cancel_binding}>(contract) }} }}"
            )?;
            ensure!(
                has_field(
                    response(&contracts[*status].1, resolver)?,
                    "state",
                    "string"
                )?,
                "async status response requires state"
            );
            writeln!(
                out,
                "fn status_state(response: &Self::StatusResponse) -> anyhow::Result<&str> {{ Ok(&response.state) }}"
            )?;
        }
        writeln!(out, "}}")?;
    }
    crate::codegen::contract_proto_modules_gen::write(out_dir, verbose, &modules)
}
