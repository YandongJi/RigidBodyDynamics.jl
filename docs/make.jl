using Documenter, RigidBodyDynamics

makedocs(
    # options
    modules = [RigidBodyDynamics]
)

deploydocs(
    deps   = Deps.pip("pygments", "mkdocs", "python-markdown-math"),
    repo = "github.com/tkoolen/RigidBodyDynamics.jl.git",
    julia  = "0.5",
    osname = "linux"
)
