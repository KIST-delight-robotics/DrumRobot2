"""
경량 StateGraph 구현 — LangGraph API 호환.

Python 3.8 + Jetson aarch64 환경에서 langgraph 패키지가 설치 불가능한 경우의
대체 구현이다. API 설계를 langgraph 와 동일하게 맞춰두었으므로,
Python 버전을 3.9 이상으로 올린 뒤에는 아래 import 두 줄만 교체하면 된다.

    # 교체 전 (현재)
    from pipeline.state_graph import StateGraph, END

    # 교체 후 (langgraph 설치 완료 시)
    from langgraph.graph import StateGraph, END

지원하는 API:
    graph = StateGraph(StateType)
    graph.add_node("name", fn)
    graph.set_entry_point("name")
    graph.add_edge("from", "to")
    graph.add_conditional_edges("from", condition_fn, {"key": "to", ...})
    app = graph.compile()
    final_state = app.invoke(initial_state)
"""

from typing import Any, Callable, Dict, Optional

# LangGraph 의 END 센티넬과 동일한 의미: 그래프 실행 종료
END = "__end__"


class _CompiledGraph:
    """
    compile() 이 반환하는 실행 가능한 그래프.
    invoke(state) 를 호출하면 entry_point 에서 시작해 END 에 도달할 때까지
    노드와 엣지를 순서대로 실행한다.
    """

    def __init__(
        self,
        nodes: Dict[str, Callable],
        edges: Dict[str, str],
        conditional_edges: Dict[str, tuple],
        entry_point: str,
    ):
        self._nodes = nodes
        self._edges = edges
        self._conditional_edges = conditional_edges
        self._entry_point = entry_point

    def invoke(self, state: Dict[str, Any]) -> Dict[str, Any]:
        """
        initial state 를 받아 그래프를 실행하고 최종 state 를 반환한다.

        실행 흐름:
        1. entry_point 노드부터 시작한다.
        2. 노드 함수가 state 를 받아 새 state 를 반환한다.
        3. conditional_edge 또는 edge 를 따라 다음 노드를 결정한다.
        4. 다음 노드가 END 이면 종료한다.
        5. 최대 100회 반복 후 강제 종료한다 (무한루프 방지).
        """
        current_node = self._entry_point
        current_state = dict(state)
        max_steps = 100

        for _ in range(max_steps):
            if current_node == END:
                break

            node_fn = self._nodes.get(current_node)
            if node_fn is None:
                raise RuntimeError(f"[StateGraph] 알 수 없는 노드: {current_node!r}")

            result = node_fn(current_state)
            if result is not None:
                current_state = dict(result)

            # 다음 노드 결정: conditional_edge 우선, 없으면 edge
            if current_node in self._conditional_edges:
                condition_fn, mapping = self._conditional_edges[current_node]
                key = condition_fn(current_state)
                next_node = mapping.get(key, END)
            elif current_node in self._edges:
                next_node = self._edges[current_node]
            else:
                # 연결된 엣지가 없으면 종료
                break

            current_node = next_node

        return current_state


class StateGraph:
    """
    LangGraph 호환 StateGraph.

    사용법:
        graph = StateGraph(MyTypedDict)
        graph.add_node("process", process_fn)
        graph.set_entry_point("process")
        graph.add_conditional_edges("process", decide_fn, {"a": "nodeA", "end": END})
        app = graph.compile()
        result = app.invoke(initial_state)
    """

    def __init__(self, state_type: Any):
        """state_type 은 TypedDict 클래스 (런타임에는 타입 힌트 용도로만 사용)."""
        self._state_type = state_type
        self._nodes: Dict[str, Callable] = {}
        self._edges: Dict[str, str] = {}
        self._conditional_edges: Dict[str, tuple] = {}
        self._entry_point: Optional[str] = None

    def add_node(self, name: str, fn: Callable) -> None:
        """이름과 처리 함수를 묶어 노드를 등록한다."""
        self._nodes[name] = fn

    def set_entry_point(self, name: str) -> None:
        """그래프 실행 시작 노드를 지정한다."""
        self._entry_point = name

    def add_edge(self, from_node: str, to_node: str) -> None:
        """무조건 전이 엣지를 추가한다."""
        self._edges[from_node] = to_node

    def add_conditional_edges(
        self,
        from_node: str,
        condition_fn: Callable,
        mapping: Dict[str, str],
    ) -> None:
        """
        조건부 전이 엣지를 추가한다.

        condition_fn(state) 의 반환값을 mapping 에서 조회해 다음 노드를 결정한다.
        반환값이 mapping 에 없으면 END 로 간다.
        """
        self._conditional_edges[from_node] = (condition_fn, mapping)

    def compile(self) -> _CompiledGraph:
        """그래프를 검증하고 실행 가능한 _CompiledGraph 를 반환한다."""
        if self._entry_point is None:
            raise RuntimeError("[StateGraph] set_entry_point() 를 먼저 호출하세요.")
        return _CompiledGraph(
            nodes=dict(self._nodes),
            edges=dict(self._edges),
            conditional_edges=dict(self._conditional_edges),
            entry_point=self._entry_point,
        )
