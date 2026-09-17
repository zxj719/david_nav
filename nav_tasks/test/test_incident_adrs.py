"""Preserve incident source text in ADRs without retaining diaries in runtime code."""

import ast
import hashlib
import io
from pathlib import Path
import re
import tokenize

import pytest

ROOT = Path(__file__).resolve().parents[2]
HISTORY = [
    ('finish-module', 'nav_tasks/docs/adr-0006-dock-thread.md',
     '018a53625b219738dff2da5306ea5f4934ef0abfbdb62edbdf96d38af0451fa1'),
    ('settle-module', 'nav_tasks/docs/adr-0006-dock-thread.md',
     '93229136745f4605dc52d9cd336b9c41030e08172d40f01ea02e5b57b778776e'),
    ('settle-spec', 'nav_tasks/docs/adr-0006-dock-thread.md',
     '7616a0fda47338b1b6380cba2c1b84fd7185451e381b80f8c53e43fb85ec176f'),
    ('settle-check', 'nav_tasks/docs/adr-0006-dock-thread.md',
     '2a8c4443651e67221540ff90c89f7330c93950d9f37dfc7ffa66c2b1a1de41d2'),
    ('settle-bridge', 'nav_tasks/docs/adr-0006-dock-thread.md',
     'c2616c2b13129714f8a02be9f6cd2b9c07eb3ea0572bbd42eec10fff0189da9a'),
    ('wire-v1-target', 'nav_executor/docs/adr-0012-v1-poi-targets.md',
     '139074b0a0e5be880c0171ac9cd989e5f9641e6bd2e40a453c4faf2dd1b2d780'),
    ('snap-success', 'nav_tasks/docs/adr-0003-goal-resolution.md',
     'ba103637fb79ad94f494fdfc16d4c35b5d5291e077156e3eb3b2f2f60c5950d8'),
    ('static-distinction', 'nav_tasks/docs/adr-0003-goal-resolution.md',
     'b92d6f9da8d3393885d7c0ceb5397e805a5c9ab6fe0733bae3fdd486ce3b60dd'),
    ('dock-future', 'nav_tasks/docs/adr-0005-ros-futures.md',
     '4447b8fdd8c50882f9162a7884e12ab3b3dada7a658e24e8080ff8671bfcef5e'),
    ('bridge-cancel', 'nav_tasks/docs/adr-0005-ros-futures.md',
     '214debc433a28f257fc88ae8a8cfff95cd4bb4cf65befa20ebe632aa69b7b541'),
]


@pytest.mark.parametrize('key,document,digest', HISTORY)
def test_adr_preserves_exact_historical_source(key, document, digest):
    """Pin selected original blocks, including punctuation, indentation and dates."""
    text = (ROOT / document).read_text()
    start = '<!-- history: ' + key + ' -->\n\n```python\n'
    assert text.count(start) == 1
    raw = text.split(start, 1)[1].split('\n```', 1)[0]
    assert hashlib.sha256(raw.encode()).hexdigest() == digest


@pytest.mark.parametrize('relative', [
    'nav_tasks/nav_tasks/skills/docking/finish.py',
    'nav_tasks/nav_tasks/skills/docking/settle.py',
    'nav_executor/nav_executor/protocol/codec.py',
])
def test_migrated_runtime_notes_use_invariants_and_adr_links(relative):
    """Inspect comments/docs only, not wire strings or runtime date parameters."""
    source = (ROOT / relative).read_text()
    tree = ast.parse(source)
    notes = [ast.get_docstring(node) or '' for node in ast.walk(tree)
             if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef,
                                  ast.AsyncFunctionDef))]
    notes += [token.string for token in tokenize.generate_tokens(io.StringIO(source).readline)
              if token.type == tokenize.COMMENT]
    text = '\n'.join(notes)
    assert '2026-09-10' not in text
    assert '2026-07-22' not in text
    assert 'adr-0006-dock-thread.md' in text or 'adr-0012-v1-poi-targets.md' in text


@pytest.mark.parametrize('document', sorted({item[1] for item in HISTORY}))
def test_adr_local_references_resolve(document):
    """Keep current implementation and evidence links independently followable."""
    path = ROOT / document
    for target in re.findall(r'\]\(([^)]+)\)', path.read_text()):
        if '://' not in target and not target.startswith('#'):
            assert (path.parent / target.split('#')[0]).exists(), target
