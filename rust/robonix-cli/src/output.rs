use colored::*;

/// Print a main action header (e.g., "Installing", "Registering")
pub fn action(action: &str, target: &str) {
    println!("{} {}", format!("[{}]", action).green().bold(), target);
}

/// Print a success completion message
pub fn success(message: &str) {
    println!("{} {}", "✓".green().bold(), message.green());
}

/// Print an info message
pub fn info(message: &str) {
    println!("{}", message);
}

/// Print a warning message
pub fn warning(message: &str) {
    println!("{} {}", "⚠".yellow().bold(), message.yellow());
}

/// Print an error message
pub fn error(message: &str) {
    eprintln!("{} {}", "✗".red().bold(), message.red());
}

/// Print a step message (like "Validating", "Processing", etc.)
pub fn step(action: &str, target: &str) {
    println!("  {} {}", format!("→ {}", action).cyan(), target);
}

/// Print a sub-step message (indented detail)
pub fn sub_step(message: &str) {
    println!("    {}", message);
}

/// Print a checkmark success message for individual items
pub fn check(message: &str) {
    println!("  {} {}", "✓".green(), message);
}

/// Print a cross error message for individual items
pub fn cross(message: &str) {
    eprintln!("  {} {}", "✗".red(), message);
}

/// Print a summary line
pub fn summary(message: &str) {
    println!("\n{}", message.dimmed());
}
