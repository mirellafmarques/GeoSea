import numpy as np

class QAManager:
    def __init__(self, max_history=100):
        self.max_history = max_history
        self.erros = []
        self.rmse_ema = 0.0
        self.alpha = 0.1
        self.last_heading = None

    def _safe_float(self, value, default=0.0):
        if value is None or np.isnan(value) or np.isinf(value):
            return default
        return float(value)

    def _default_metrics(self):
        return {
            "gnss": {}, "ekf": {}, "navegacao": {}, "estatisticas": {},
            "alertas": {"flags": ["System Initializing"]} }

    def update(self, pos_gnss, pos_ekf, accuracy=None, alt_gnss=0.0, fix=None, satellites=None, 
               hdop=None, pdop=None, vdop=None, covariance=None, innovation=None, nis=None, chi2_threshold=None,
               bias=None, corrente_est=None, xte=None, heading=None, bearing=None, cog=None, 
               distancia=None, eta=None, velocidade=None):

        if pos_gnss is None or pos_ekf is None:
            return self._default_metrics()

        # GNSS and FKE
        pos_gnss = np.array(pos_gnss, dtype=float)
        pos_ekf = np.array(pos_ekf, dtype=float)

        if np.any(np.isnan(pos_gnss)) or np.any(np.isnan(pos_ekf)):
            return self._default_metrics()

        # Historical Calculation
        erro = np.linalg.norm(pos_gnss - pos_ekf)
        self.erros.append(erro)
        if len(self.erros) > self.max_history:
            self.erros.pop(0)

        erro2 = erro ** 2
        self.rmse_ema = (self.alpha * erro2 + (1 - self.alpha) * self.rmse_ema)
        rmse = np.sqrt(self.rmse_ema)

        stability = np.std(self.erros[-5:]) if len(self.erros) >= 5 else 0.0
        trend = np.mean(np.diff(self.erros[-10:])) if len(self.erros) >= 10 else 0.0
        
        # Statistic
        erro_medio = np.mean(self.erros)
        erro_maximo = np.max(self.erros)
        erro_minimo = np.min(self.erros)
        # Calculates the P95 only if there is sufficient history
        p95 = np.percentile(self.erros, 95) if len(self.erros) >= 20 else erro

        # Diference in heading
        heading_diff = 0.0
        if heading is not None and not np.isnan(heading):
            if self.last_heading is not None:
                diff = heading - self.last_heading
                diff = (diff + 180) % 360 - 180
                heading_diff = abs(diff)
            self.last_heading = heading

        # Extraction of xte, nis, and chi2 parameters and covariance
        safe_acc = self._safe_float(accuracy, default=10.0)
        safe_xte = self._safe_float(xte)
        safe_nis = self._safe_float(nis)
        safe_threshold = self._safe_float(chi2_threshold, default=5.991)
        covariance_trace = 0.0
        if covariance is not None:
            covariance_trace = float(np.trace(covariance))

        # Creating flags
        flags = []
        if satellites is not None and satellites < 6: flags.append("LOW_SATELLITES")
        if safe_nis > safe_threshold: flags.append("HIGH_NIS")
        if covariance_trace > 10.0: flags.append("HIGH_COVARIANCE") 
        if fix is not None:
            if fix == 0:
                flags.append("GNSS_INVALID") # 0 = Invalid fix 
            elif fix == 6:
                flags.append("GNSS_ESTIMATED") # 6 = Dead reckoning 
        if xte is not None and abs(safe_xte) > 5.0: flags.append("OUT_OF_TRACK")
        if heading_diff > 15: flags.append("HIGH_HEADING_ERROR")

        dados = {
            "gnss": {
                "accuracy": safe_acc,
                "alt_gnss": self._safe_float(alt_gnss),
                "fix": fix,
                "satellites": satellites,
                "hdop": self._safe_float(hdop),
                "pdop": self._safe_float(pdop),
                "vdop": self._safe_float(vdop)   },
            "ekf": {
                "covariance_trace": covariance_trace,
                "innovation": innovation if innovation is not None else np.zeros(2).tolist(),
                "nis": safe_nis,
                "chi2_threshold": safe_threshold,
                "bias": bias,
                "corrente_est": corrente_est   },
            "navegacao": {
                "xte": safe_xte,
                "heading": self._safe_float(heading),
                "heading_diff": float(heading_diff),
                "bearing": self._safe_float(bearing),
                "cog": self._safe_float(cog),
                "distancia": self._safe_float(distancia),
                "eta": eta,
                "velocidade": self._safe_float(velocidade)   },
            "estatisticas": {
                "erro_atual": float(erro),
                "rmse_ema": float(rmse),
                "erro_medio": float(erro_medio),
                "erro_maximo": float(erro_maximo),
                "erro_minimo": float(erro_minimo),
                "p95": float(p95),
                "trend": float(trend),
                "stability": float(stability)  },
            "alertas": {
                "flags": flags  }
        }        
        return dados

    @staticmethod
    def flatten_for_export(dados_agrupados):
        flat_dict = {}
        for categoria, metricas in dados_agrupados.items():
            if categoria == "alertas":
                flat_dict["flags_alerta"] = ";".join(metricas["flags"]) if metricas.get("flags") else "OK"
            else:
                for chave, valor in metricas.items():
                    flat_dict[chave] = valor                    
        return flat_dict

    def reset(self):
        self.erros = []
        self.rmse_ema = 0.0
        self.last_heading = None